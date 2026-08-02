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
    // 复位 & DRAM 初始化 (合并到一个 initial 块避免竞争)
    // ============================================================
    initial begin
        aresetn = 1'b0;
        arready = 1'b0; awready = 1'b0; wready  = 1'b0;
        rvalid  = 1'b0; rlast   = 1'b0; rresp   = 2'b0;
        bvalid  = 1'b0; bresp   = 2'b0;
        rid     = 4'b0; rdata   = 32'b0;
        bid     = 4'b0;

        $readmemh("bench_core.mem", dram);
        $display("[TB] DRAM loaded, %0d words", DRAM_DEPTH);

        #(CLK_PERIOD);
        aresetn = 1'b1;
    end

    // ============================================================
    // 读通道 (单进程 FSM，全部寄存器输出)
    //
    //   IDLE: arready=1. arvalid & arready → 锁存 addr/id → RESP
    //   RESP: arready=0, rvalid=1, rdata = 组合读 dram[latched_addr]
    //         rvalid & rready → IDLE
    //
    //   关键: rdata 用 assign 组合读 dram[r_addr_latch], 与 rvalid≤1
    //        在同一 NBA 更新, 彻底消除寄存器预读引入的 delta-cycle 偏移.
    // ============================================================
    localparam R_IDLE = 1'b0, R_RESP = 1'b1;
    reg r_state;
    reg [31:0] r_addr_latch;
    reg [ 3:0] r_id_latch;

    // 组合读: r_data_comb 跟随 r_addr_latch 即时变化
    wire [31:0] r_data_comb;
    assign r_data_comb = dram[r_addr_latch[DRAM_AW+1:2]];

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
        end else begin
            case (r_state)
                R_IDLE: begin
                    arready <= 1'b1;
                    if (arvalid && arready) begin
                        r_addr_latch <= araddr;
                        r_id_latch   <= arid;
                        r_state      <= R_RESP;
                    end
                end

                R_RESP: begin
                    arready <= 1'b0;
                    rvalid  <= 1'b1;
                    rlast   <= 1'b1;
                    rresp   <= 2'b00;
                    rid     <= r_id_latch;
                    rdata   <= r_data_comb;
                    if (rvalid && rready) begin
                        rvalid  <= 1'b0;
                        r_state <= R_IDLE;
                    end
                end

                default: r_state <= R_IDLE;
            endcase
        end
    end

    // ============================================================
    // 写通道 (单进程 FSM，全部寄存器输出)
    //
    //   IDLE: awready=1, wready=1
    //         分别锁存 AW 和 W (可同拍或不同拍)
    //         AW+W 都就绪 → 写 DRAM → RESP
    //   RESP: bvalid=1, awready=0, wready=0
    //         bvalid & bready → IDLE
    //
    //   写数据选择: 若 W 已锁存则用 wd_latch, 否则用当拍 wdata
    //   写地址选择: 若 AW 已锁存则用 aw_latch, 否则用当拍 awaddr
    //   (非阻塞赋值规则: RHS 全部在时钟沿前求值, 因此用 flag 做 mux)
    // ============================================================
    localparam W_IDLE = 2'd0, W_RESP = 2'd1;
    reg [1:0] w_state;

    reg        aw_got, w_got;
    reg [31:0] aw_latch;
    reg [ 3:0] awid_latch;
    reg [31:0] wd_latch;

    // 写使能计算 (IDLE 态时, AW 和 W 是否都已就绪)
    wire w_both_ready = (aw_got || (awvalid && awready)) &&
                        (w_got  || (wvalid && wready));

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            w_state    <= W_IDLE;
            awready    <= 1'b0;
            wready     <= 1'b0;
            bvalid     <= 1'b0;
            bresp      <= 2'b0;
            bid        <= 4'b0;
            aw_got     <= 1'b0;
            w_got      <= 1'b0;
            aw_latch   <= 32'b0;
            awid_latch <= 4'b0;
            wd_latch   <= 32'b0;
        end else begin
            case (w_state)
                W_IDLE: begin
                    awready <= 1'b1;
                    wready  <= 1'b1;
                    bvalid  <= 1'b0;

                    // 锁存 AW
                    if (awvalid && awready && !aw_got) begin
                        aw_latch   <= awaddr;
                        awid_latch <= awid;
                        aw_got     <= 1'b1;
                    end

                    // 锁存 W
                    if (wvalid && wready && !w_got) begin
                        wd_latch <= wdata;
                        w_got    <= 1'b1;
                    end

                    // 两个都就绪 → 写入 DRAM, 进入 RESP
                    if (w_both_ready) begin
                        // 地址: aw_got=1 用锁存, =0 用当拍
                        dram[aw_got ? aw_latch[DRAM_AW+1:2]
                                    : awaddr[DRAM_AW+1:2]]
                            <= w_got ? wd_latch : wdata;

                        aw_got  <= 1'b0;
                        w_got   <= 1'b0;
                        bid     <= awid_latch;
                        bresp   <= 2'b00;
                        w_state <= W_RESP;
                    end
                end

                W_RESP: begin
                    awready <= 1'b0;
                    wready  <= 1'b0;
                    bvalid  <= 1'b1;
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
