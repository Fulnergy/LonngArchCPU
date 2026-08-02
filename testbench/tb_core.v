// ============================================================================
// tb_core — core_top 集成测试平台 (带 AXI DRAM 模拟)
// ============================================================================
//   模拟 64KB DRAM (0x0000~0xFFFF)，BRAM 初始化为 bench_core.mem。
//   通过 AXI4-Lite 接口与 core_top 通信。1000 周期后 $finish。
//
//   所有 AXI slave 输出均为寄存器输出，无组合逻辑环风险。
// ============================================================================

`timescale 1ns / 1ps

module tb_core;

    parameter CLK_PERIOD  = 10;
    parameter SIM_CYCLES  = 10000;
    parameter DRAM_DEPTH  = 16384;        // 64KB / 4B
    parameter DRAM_AW     = 14;           // addr[15:2]

    // ============================================================
    // 时钟 & 复位
    // ============================================================
    reg aclk;
    reg aresetn;

    // ============================================================
    // AXI 信号 — Master→Slave (wire)
    // ============================================================
    wire [ 3:0] arid,   awid,   wid;
    wire [31:0] araddr, awaddr, wdata;
    wire [ 7:0] arlen,  awlen;
    wire [ 2:0] arsize, awsize;
    wire [ 1:0] arburst,awburst,arlock,awlock;
    wire [ 3:0] arcache,awcache,wstrb;
    wire [ 2:0] arprot, awprot;
    wire        arvalid, awvalid, wvalid, wlast;
    wire        rready,  bready;

    // AXI 信号 — Slave→Master (reg, 全部寄存器输出)
    reg  [ 3:0] rid;
    reg  [31:0] rdata;
    reg  [ 1:0] rresp;
    reg         rlast, rvalid;
    reg         arready;

    reg  [ 3:0] bid;
    reg  [ 1:0] bresp;
    reg         bvalid;
    reg         awready, wready;

    // ============================================================
    // DRAM (BRAM)
    // ============================================================
    reg [31:0] dram [0:DRAM_DEPTH-1];

    // ============================================================
    // core_top 例化
    // ============================================================
    core_top u_core_top (
        .aclk      (aclk),
        .aresetn   (aresetn),
        .intrpt    (8'b0),

        .arid      (arid),      .araddr    (araddr),
        .arlen     (arlen),     .arsize    (arsize),
        .arburst   (arburst),   .arlock    (arlock),
        .arcache   (arcache),   .arprot    (arprot),
        .arvalid   (arvalid),   .arready   (arready),
        .rid       (rid),       .rdata     (rdata),
        .rresp     (rresp),     .rlast     (rlast),
        .rvalid    (rvalid),    .rready    (rready),

        .awid      (awid),      .awaddr    (awaddr),
        .awlen     (awlen),     .awsize    (awsize),
        .awburst   (awburst),   .awlock    (awlock),
        .awcache   (awcache),   .awprot    (awprot),
        .awvalid   (awvalid),   .awready   (awready),
        .wid       (wid),       .wdata     (wdata),
        .wstrb     (wstrb),     .wlast     (wlast),
        .wvalid    (wvalid),    .wready    (wready),
        .bid       (bid),       .bresp     (bresp),
        .bvalid    (bvalid),    .bready    (bready),

        .break_point      (1'b0),
        .infor_flag       (1'b0),
        .reg_num          (5'b0),
        .ws_valid         (),    .rf_rdata         (),
        .debug0_wb_pc     (),    .debug0_wb_rf_wen (),
        .debug0_wb_rf_wnum(),    .debug0_wb_rf_wdata()
    );

    // ============================================================
    // 时钟
    // ============================================================
    initial aclk = 0;
    always #(CLK_PERIOD / 2) aclk = ~aclk;

    // ============================================================
    // 复位 & DRAM 初始化
    // ============================================================
    integer dram_init_i;
    initial begin
        aresetn = 1'b0;
        arready = 1'b0; awready = 1'b0; wready  = 1'b0;
        rvalid  = 1'b0; rlast   = 1'b0; rresp   = 2'b0;
        bvalid  = 1'b0; bresp   = 2'b0;
        rid     = 4'b0; rdata   = 32'b0;
        bid     = 4'b0;

        // 先全部清零, 避免未初始化区域为 X
        for (dram_init_i = 0; dram_init_i < DRAM_DEPTH; dram_init_i = dram_init_i + 1)
            dram[dram_init_i] = 32'b0;

        $readmemh("bench_core.mem", dram);
        $display("[TB] DRAM loaded, %0d words", DRAM_DEPTH);

        #(CLK_PERIOD);
        aresetn = 1'b1;
    end

    // ============================================================
    // 读通道 (支持 burst: arlen 控制返回节拍数)
    //
    //   IDLE: arready=1, AR握手 → 锁存 addr/id/len → RESP
    //   RESP: 逐拍返回数据, rlast 在末拍置 1, 地址自动递增
    // ============================================================
    localparam R_IDLE = 1'b0, R_RESP = 1'b1;
    reg r_state;
    reg [31:0] r_addr_latch;
    reg [ 3:0] r_id_latch;
    reg [ 7:0] r_len;             // burst 长度 (arlen 锁存值)
    reg [ 7:0] r_cnt;             // 当前 beat 计数 (0 ~ arlen)

    // 组合读: r_data_comb 跟随 r_addr_latch 即时变化
    wire [DRAM_AW-1:0] r_dram_idx;
    assign r_dram_idx = r_addr_latch[DRAM_AW+1:2];
    wire [31:0] r_data_comb;
    assign r_data_comb = dram[r_dram_idx];

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            r_state      <= R_IDLE;
            arready      <= 1'b0;
            rvalid       <= 1'b0;
            rlast        <= 1'b0;
            rresp        <= 2'b0;
            rid          <= 4'b0;
            rdata        <= 32'b0;
            r_addr_latch <= 32'b0;
            r_id_latch   <= 4'b0;
            r_len        <= 8'd0;
            r_cnt        <= 8'd0;
        end else begin
            case (r_state)
                R_IDLE: begin
                    arready <= 1'b1;
                    rvalid  <= 1'b0;
                    if (arvalid && arready) begin
                        r_addr_latch <= araddr;
                        r_id_latch   <= arid;
                        r_len        <= arlen;
                        r_cnt        <= 8'd0;
                        r_state      <= R_RESP;
                    end
                end

                R_RESP: begin
                    arready <= 1'b0;
                    rvalid  <= 1'b1;
                    rresp   <= 2'b00;
                    rid     <= r_id_latch;
                    rdata   <= r_data_comb;
                    rlast   <= (r_cnt == r_len);

                    if (rvalid && rready) begin
                        r_addr_latch <= r_addr_latch + 32'd4;
                        r_cnt        <= r_cnt + 8'd1;
                        if (r_cnt == r_len) begin
                            rvalid  <= 1'b0;
                            r_state <= R_IDLE;
                        end
                    end
                end

                default: r_state <= R_IDLE;
            endcase
        end
    end

    // ============================================================
    // 写通道 (支持 burst: awlen 控制接收节拍数)
    //
    //   IDLE: awready=1, wready=1. AW 握手锁存 addr/len.
    //         连续接收 W beats 写入 DRAM, 地址自增.
    //         wlast=1 → 进入 RESP
    //   RESP: bvalid=1, 等待 bready → IDLE
    // ============================================================
    localparam W_IDLE = 2'd0, W_RESP = 2'd1;
    reg [1:0] w_state;

    reg        aw_done;          // AW 已握手
    reg [31:0] w_addr_latch;     // 当前写地址
    reg [ 7:0] w_len_latch;      // burst 长度
    reg [ 7:0] w_cnt;            // 当前 beat 计数

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            w_state      <= W_IDLE;
            awready      <= 1'b0;
            wready       <= 1'b0;
            bvalid       <= 1'b0;
            bresp        <= 2'b0;
            bid          <= 4'b0;
            aw_done      <= 1'b0;
            w_addr_latch <= 32'b0;
            w_len_latch  <= 8'd0;
            w_cnt        <= 8'd0;
        end else begin
            case (w_state)
                W_IDLE: begin
                    awready <= !aw_done;
                    wready  <= 1'b1;
                    bvalid  <= 1'b0;

                    // AW 握手
                    if (awvalid && awready && !aw_done) begin
                        w_addr_latch <= awaddr;
                        w_len_latch  <= awlen;
                        w_cnt        <= 8'd0;
                        aw_done      <= 1'b1;
                    end

                    // W 握手: 逐拍写入 DRAM, 地址自增
                    if (wvalid && wready) begin
                        dram[w_addr_latch[DRAM_AW+1:2]] <= wdata;
                        w_addr_latch <= w_addr_latch + 32'd4;
                        w_cnt        <= w_cnt + 8'd1;

                        if (wlast) begin
                            aw_done  <= 1'b0;
                            w_state  <= W_RESP;
                        end
                    end
                end

                W_RESP: begin
                    awready <= 1'b0;
                    wready  <= 1'b0;
                    bvalid  <= 1'b1;
                    bresp   <= 2'b00;
                    bid     <= 4'd1;
                    if (bvalid && bready) begin
                        bvalid  <= 1'b0;
                        w_state <= W_IDLE;
                    end
                end

                default: w_state <= W_IDLE;
            endcase
        end
    end

    // ============================================================
    // 仿真控制
    // ============================================================
    integer cycle_cnt;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn)
            cycle_cnt <= 0;
        else
            cycle_cnt <= cycle_cnt + 1;
    end

    initial begin
        #(CLK_PERIOD * SIM_CYCLES);
        $display("============================================================");
        $display("[TB] Simulation finished after %0d cycles", SIM_CYCLES);
        $display("============================================================");
        $finish;
    end

endmodule
