// ============================================================================
// axi_bridge — AXI 转接桥 (iCache + dCache → AXI)
// ============================================================================
//   LoongArch 规范:
//     - arlen/awlen = 0 (单拍, 无 burst)
//     - arid: 0=取指(iCache), 1=取数(dCache读)
//     - awid/wid: 1=存数(dCache写), wlast=1
//     - arcache/awcache = 0, arprot/awprot = 0
//
//   Cache line fill: 每 word 独立 AXI 读事务 (16次 × 单拍)
//   Cache write-back: 每 word 独立 AXI 写事务 (16次 × 单拍)
//   仲裁: dCache 优先于 iCache
// ============================================================================

module axi_bridge (
    input  wire         clk,
    input  wire         rst_n,

    // ============================================================
    // iCache 侧 (只读)
    // ============================================================
    input  wire         i_ext_req,
    input  wire [31:0]  i_ext_addr,
    output reg  [31:0]  i_ext_rdata,
    output reg          i_ext_ready,

    // ============================================================
    // dCache 侧 (读写)
    // ============================================================
    input  wire         d_ext_req,
    input  wire         d_ext_we,
    input  wire [31:0]  d_ext_addr,
    input  wire [31:0]  d_ext_wdata,
    input  wire [3:0]   d_ext_wstrb,
    output reg  [31:0]  d_ext_rdata,
    output reg          d_ext_ready,

    // ============================================================
    // AXI Read Address
    // ============================================================
    output reg  [ 3:0]  arid,
    output reg  [31:0]  araddr,
    output reg  [ 7:0]  arlen,
    output reg  [ 2:0]  arsize,
    output reg  [ 1:0]  arburst,
    output reg  [ 1:0]  arlock,
    output reg  [ 3:0]  arcache,
    output reg  [ 2:0]  arprot,
    output reg          arvalid,
    input  wire         arready,

    // ============================================================
    // AXI Read Data
    // ============================================================
    input  wire [ 3:0]  rid,
    input  wire [31:0]  rdata,
    input  wire [ 1:0]  rresp,
    input  wire         rlast,
    input  wire         rvalid,
    output reg          rready,

    // ============================================================
    // AXI Write Address
    // ============================================================
    output reg  [ 3:0]  awid,
    output reg  [31:0]  awaddr,
    output reg  [ 7:0]  awlen,
    output reg  [ 2:0]  awsize,
    output reg  [ 1:0]  awburst,
    output reg  [ 1:0]  awlock,
    output reg  [ 3:0]  awcache,
    output reg  [ 2:0]  awprot,
    output reg          awvalid,
    input  wire         awready,

    // ============================================================
    // AXI Write Data
    // ============================================================
    output reg  [ 3:0]  wid,
    output reg  [31:0]  wdata,
    output reg  [ 3:0]  wstrb,
    output reg          wlast,
    output reg          wvalid,
    input  wire         wready,

    // ============================================================
    // AXI Write Response
    // ============================================================
    input  wire [ 3:0]  bid,
    input  wire [ 1:0]  bresp,
    input  wire         bvalid,
    output reg          bready
);

    // ============================================================
    // AXI 固定参数 (LoongArch 规范)
    // ============================================================
    localparam AXI_LEN_SINGLE = 8'd0;        // 单拍
    localparam AXI_SIZE_WORD  = 3'b010;      // 4 bytes
    localparam AXI_BURST_INCR = 2'b01;
    localparam ID_IFETCH      = 4'd0;        // iCache 取指
    localparam ID_DACCESS     = 4'd1;        // dCache 存取

    // ============================================================
    // 状态机
    // ============================================================
    reg [2:0] state, next_state;

    localparam S_IDLE      = 3'd0;
    localparam S_AR_REQ    = 3'd1;
    localparam S_R_DATA    = 3'd2;
    localparam S_AW_REQ    = 3'd3;
    localparam S_W_DATA    = 3'd4;
    localparam S_B_RESP    = 3'd5;

    // ============================================================
    // 仲裁: dCache 优先, 单拍事务 (无 burst, 无缓冲)
    // ============================================================
    reg  is_dcache;          // 当前事务: 1=dCache, 0=iCache
    reg  is_write;           // 当前事务: 1=写, 0=读

    wire d_take = d_ext_req;
    wire i_take = i_ext_req && !d_ext_req;

    always @(posedge clk) begin
        if (state == S_IDLE) begin
            if (d_take) begin
                is_dcache <= 1'b1;
                is_write  <= d_ext_we;
            end else if (i_take) begin
                is_dcache <= 1'b0;
                is_write  <= 1'b0;
            end
        end
    end

    // ============================================================
    // cache ext 接口 (统一驱动, 单拍直通)
    // ============================================================
    always @(*) begin
        i_ext_ready = 1'b0;
        i_ext_rdata = 32'b0;
        d_ext_ready = 1'b0;
        d_ext_rdata = 32'b0;

        case (state)
            S_R_DATA: begin
                if (rvalid) begin
                    if (is_dcache) begin
                        d_ext_ready = 1'b1;
                        d_ext_rdata = rdata;
                    end else begin
                        i_ext_ready = 1'b1;
                        i_ext_rdata = rdata;
                    end
                end
            end

            S_W_DATA: begin
                if (is_write && d_ext_req && d_ext_we)
                    d_ext_ready = wready;
            end

            default: ;
        endcase
    end

    // ============================================================
    // 状态转移
    // ============================================================
    always @(posedge clk) begin
        if (!rst_n)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    always @(*) begin
        next_state = state;

        case (state)
            S_IDLE: begin
                if (d_take && d_ext_we)
                    next_state = S_AW_REQ;          // dCache write-back
                else if (d_take || i_take)
                    next_state = S_AR_REQ;          // fill
            end

            S_AR_REQ: begin
                if (arready)
                    next_state = S_R_DATA;
            end

            S_R_DATA: begin
                if (rvalid)
                    next_state = S_IDLE;            // 单拍完成
            end

            S_AW_REQ: begin
                if (awready)
                    next_state = S_W_DATA;
            end

            S_W_DATA: begin
                if (wready)
                    next_state = S_B_RESP;
            end

            S_B_RESP: begin
                if (bvalid)
                    next_state = S_IDLE;
            end

            default: next_state = S_IDLE;
        endcase
    end

    // ============================================================
    // AXI AR 通道 (单拍读, arid 区分取指/取数)
    // ============================================================
    always @(*) begin
        arid    = is_dcache ? ID_DACCESS : ID_IFETCH;
        araddr  = is_dcache ? d_ext_addr : i_ext_addr;
        arlen   = AXI_LEN_SINGLE;
        arsize  = AXI_SIZE_WORD;
        arburst = AXI_BURST_INCR;
        arlock  = 2'b00;
        arcache = 4'b0000;
        arprot  = 3'b000;
        arvalid = (state == S_AR_REQ);
    end

    // ============================================================
    // AXI R 通道
    // ============================================================
    always @(*) begin
        rready = (state == S_R_DATA);
    end

    // ============================================================
    // AXI AW 通道 (单拍写, awid=1)
    // ============================================================
    always @(*) begin
        awid    = ID_DACCESS;
        awaddr  = d_ext_addr;
        awlen   = AXI_LEN_SINGLE;
        awsize  = AXI_SIZE_WORD;
        awburst = AXI_BURST_INCR;
        awlock  = 2'b00;
        awcache = 4'b0000;
        awprot  = 3'b000;
        awvalid = (state == S_AW_REQ);
    end

    // ============================================================
    // AXI W 通道 (单拍写, wid=1, wlast=1)
    // ============================================================
    always @(*) begin
        wid    = ID_DACCESS;
        wdata  = d_ext_wdata;
        wstrb  = d_ext_wstrb;
        wlast  = 1'b1;                              // 单拍恒为最后一拍
        wvalid = (state == S_W_DATA) && d_ext_req && d_ext_we;
    end

    // ============================================================
    // AXI B 通道
    // ============================================================
    always @(*) begin
        bready = (state == S_B_RESP);
    end

endmodule
