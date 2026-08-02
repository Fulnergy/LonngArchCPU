// ============================================================================
// axi_bridge — AXI 转接桥 (纯组合 MUX + 仲裁)
// ============================================================================
//   iCache / dCache 各自输出简化 AXI 子集, bridge 负责:
//     - 仲裁: dCache 优先级高于 iCache
//     - MUX: 将选中 cache 的 AXI 信号直通到顶层
//     - 状态: 仅 d_burst_active 寄存器跟踪 dCache burst 占用
//
//   与旧版区别:
//     - 无 FSM, 无 ext_* 协议转换
//     - 支持 burst (arlen/awlen 由 cache 控制, 通常=15)
//     - 除 d_burst_active 外所有输出均为纯组合逻辑
// ============================================================================

module axi_bridge (
    input  wire         clk,
    input  wire         rst_n,

    // ============================================================
    // iCache 侧 (只读 — AR + R)
    // ============================================================
    input  wire         i_arvalid,
    input  wire [31:0]  i_araddr,
    input  wire [ 7:0]  i_arlen,
    input  wire [ 2:0]  i_arsize,
    input  wire [ 1:0]  i_arburst,
    output wire         i_arready,

    output wire         i_rvalid,
    output wire [31:0]  i_rdata,
    output wire         i_rlast,
    output wire [ 1:0]  i_rresp,
    input  wire         i_rready,

    // ============================================================
    // dCache 侧 (读写 — AR + R + AW + W + B)
    // ============================================================
    input  wire         d_arvalid,
    input  wire [31:0]  d_araddr,
    input  wire [ 7:0]  d_arlen,
    input  wire [ 2:0]  d_arsize,
    input  wire [ 1:0]  d_arburst,
    output wire         d_arready,

    output wire         d_rvalid,
    output wire [31:0]  d_rdata,
    output wire         d_rlast,
    output wire [ 1:0]  d_rresp,
    input  wire         d_rready,

    input  wire         d_awvalid,
    input  wire [31:0]  d_awaddr,
    input  wire [ 7:0]  d_awlen,
    input  wire [ 2:0]  d_awsize,
    input  wire [ 1:0]  d_awburst,
    output wire         d_awready,

    input  wire         d_wvalid,
    input  wire [31:0]  d_wdata,
    input  wire [ 3:0]  d_wstrb,
    input  wire         d_wlast,
    output wire         d_wready,

    output wire         d_bvalid,
    output wire [ 1:0]  d_bresp,
    input  wire         d_bready,

    // ============================================================
    // AXI Read Address (顶层)
    // ============================================================
    output wire [ 3:0]  arid,
    output wire [31:0]  araddr,
    output wire [ 7:0]  arlen,
    output wire [ 2:0]  arsize,
    output wire [ 1:0]  arburst,
    output wire [ 1:0]  arlock,
    output wire [ 3:0]  arcache,
    output wire [ 2:0]  arprot,
    output wire         arvalid,
    input  wire         arready,

    // ============================================================
    // AXI Read Data (顶层)
    // ============================================================
    input  wire [ 3:0]  rid,
    input  wire [31:0]  rdata,
    input  wire [ 1:0]  rresp,
    input  wire         rlast,
    input  wire         rvalid,
    output wire         rready,

    // ============================================================
    // AXI Write Address (顶层)
    // ============================================================
    output wire [ 3:0]  awid,
    output wire [31:0]  awaddr,
    output wire [ 7:0]  awlen,
    output wire [ 2:0]  awsize,
    output wire [ 1:0]  awburst,
    output wire [ 1:0]  awlock,
    output wire [ 3:0]  awcache,
    output wire [ 2:0]  awprot,
    output wire         awvalid,
    input  wire         awready,

    // ============================================================
    // AXI Write Data (顶层)
    // ============================================================
    output wire [ 3:0]  wid,
    output wire [31:0]  wdata,
    output wire [ 3:0]  wstrb,
    output wire         wlast,
    output wire         wvalid,
    input  wire         wready,

    // ============================================================
    // AXI Write Response (顶层)
    // ============================================================
    input  wire [ 3:0]  bid,
    input  wire [ 1:0]  bresp,
    input  wire         bvalid,
    output wire         bready
);

    // ============================================================
    // 仲裁状态 — 唯一寄存器
    //   d_burst_active: dCache 正在占用总线 (burst 进行中)
    //   置位: dCache AR 或 AW 握手成功
    //   清零: dCache R burst 完成 (rlast) 或 B 响应收到
    // ============================================================
    reg d_burst_active;

    always @(posedge clk) begin
        if (!rst_n) begin
            d_burst_active <= 1'b0;
        end else begin
            if (d_arvalid && d_arready)
                d_burst_active <= 1'b1;
            else if (d_awvalid && d_awready)
                d_burst_active <= 1'b1;
            else if (d_burst_active && d_rvalid && d_rready && d_rlast)
                d_burst_active <= 1'b0;
            else if (d_burst_active && d_bvalid && d_bready)
                d_burst_active <= 1'b0;
        end
    end

    // dCache 当前是否 "拥有" AR 通道
    wire d_ar_active;
    assign d_ar_active = d_arvalid || d_awvalid || d_burst_active;

    // ============================================================
    // AR Channel — MUX: dCache > iCache
    // ============================================================
    assign arid    = d_ar_active ? 4'd1 : 4'd0;
    assign araddr  = d_ar_active ? d_araddr  : i_araddr;
    assign arlen   = d_ar_active ? d_arlen   : i_arlen;
    assign arsize  = d_ar_active ? d_arsize  : i_arsize;
    assign arburst = d_ar_active ? d_arburst : i_arburst;
    assign arlock  = 2'b00;
    assign arcache = 4'b0000;
    assign arprot  = 3'b000;
    assign arvalid = d_ar_active ? d_arvalid : i_arvalid;

    assign i_arready = !d_ar_active && arready;
    assign d_arready =  d_ar_active && arready;

    // ============================================================
    // R Channel — DEMUX: 根据 d_burst_active 路由返回数据
    //   注意: 必须用 d_burst_active (而非 d_ar_active),
    //   因为 AR 握手后 d_arvalid 撤销但 R 数据还在返回中
    // ============================================================
    assign i_rvalid = !d_burst_active && rvalid;
    assign i_rdata  = rdata;
    assign i_rlast  = rlast;
    assign i_rresp  = rresp;

    assign d_rvalid = d_burst_active && rvalid;
    assign d_rdata  = rdata;
    assign d_rlast  = rlast;
    assign d_rresp  = rresp;

    assign rready = d_burst_active ? d_rready : i_rready;

    // ============================================================
    // AW Channel — 直通 (仅 dCache)
    // ============================================================
    assign awid    = 4'd1;
    assign awaddr  = d_awaddr;
    assign awlen   = d_awlen;
    assign awsize  = d_awsize;
    assign awburst = d_awburst;
    assign awlock  = 2'b00;
    assign awcache = 4'b0000;
    assign awprot  = 3'b000;
    assign awvalid = d_awvalid;

    assign d_awready = awready;

    // ============================================================
    // W Channel — 直通 (仅 dCache)
    // ============================================================
    assign wid    = 4'd1;
    assign wdata  = d_wdata;
    assign wstrb  = d_wstrb;
    assign wlast  = d_wlast;
    assign wvalid = d_wvalid;

    assign d_wready = wready;

    // ============================================================
    // B Channel — 直通 (仅 dCache)
    // ============================================================
    assign d_bvalid = bvalid;
    assign d_bresp  = bresp;

    assign bready = d_bready;

endmodule
