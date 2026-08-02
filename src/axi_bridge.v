// ============================================================================
// axi_bridge — AXI 仲裁+MUX (纯连线, 无状态机)
// ============================================================================
//   i/d Cache 直出简化 AXI burst, 桥仅做仲裁 + 信号 MUX/DEMUX.
//   burst 参数 (arburst/awburst/lock/cache/prot) 桥内部固定补齐.
//
//   仲裁: dCache 优先, owner 事务锁定, burst 结束时释放.
// ============================================================================

module axi_bridge (
    input  wire         clk,
    input  wire         rst_n,

    // ============================================================
    // iCache 侧 AXI-R (只读)
    // ============================================================
    input  wire         i_arvalid,
    input  wire [31:0]  i_araddr,
    input  wire [ 7:0]  i_arlen,
    input  wire [ 2:0]  i_arsize,
    output wire         i_arready,
    output wire         i_rvalid,
    output wire [31:0]  i_rdata,
    output wire [ 1:0]  i_rresp,
    output wire         i_rlast,
    input  wire         i_rready,

    // ============================================================
    // dCache 侧 AXI-R (读)
    // ============================================================
    input  wire         d_arvalid,
    input  wire [31:0]  d_araddr,
    input  wire [ 7:0]  d_arlen,
    input  wire [ 2:0]  d_arsize,
    output wire         d_arready,
    output wire         d_rvalid,
    output wire [31:0]  d_rdata,
    output wire [ 1:0]  d_rresp,
    output wire         d_rlast,
    input  wire         d_rready,

    // ============================================================
    // dCache 侧 AXI-W (写)
    // ============================================================
    input  wire         d_awvalid,
    input  wire [31:0]  d_awaddr,
    input  wire [ 7:0]  d_awlen,
    input  wire [ 2:0]  d_awsize,
    output wire         d_awready,
    input  wire         d_wvalid,
    input  wire [31:0]  d_wdata,
    input  wire [ 3:0]  d_wstrb,
    input  wire         d_wlast,
    output wire         d_wready,
    output wire         d_bvalid,
    output wire [ 1:0]  d_bresp,
    output wire [ 3:0]  d_bid,
    input  wire         d_bready,

    // ============================================================
    // AXI 外部 (合并后)
    // ============================================================
    output wire [ 3:0]  arid,    output wire [31:0]  araddr,
    output wire [ 7:0]  arlen,   output wire [ 2:0]  arsize,
    output wire [ 1:0]  arburst, output wire [ 1:0]  arlock,
    output wire [ 3:0]  arcache, output wire [ 2:0]  arprot,
    output wire         arvalid, input  wire         arready,
    input  wire [ 3:0]  rid,     input  wire [31:0]  rdata,
    input  wire [ 1:0]  rresp,   input  wire         rlast,
    input  wire         rvalid,  output wire         rready,
    output wire [ 3:0]  awid,    output wire [31:0]  awaddr,
    output wire [ 7:0]  awlen,   output wire [ 2:0]  awsize,
    output wire [ 1:0]  awburst, output wire [ 1:0]  awlock,
    output wire [ 3:0]  awcache, output wire [ 2:0]  awprot,
    output wire         awvalid, input  wire         awready,
    output wire [ 3:0]  wid,     output wire [31:0]  wdata,
    output wire [ 3:0]  wstrb,   output wire         wlast,
    output wire         wvalid,  input  wire         wready,
    input  wire [ 3:0]  bid,     input  wire [ 1:0]  bresp,
    input  wire         bvalid,  output wire         bready
);

    localparam ID_IFETCH  = 4'd0;
    localparam ID_DACCESS = 4'd1;

    // ============================================================
    // 仲裁: owner 事务锁定
    //   0=空闲, 1=iCache, 2=dCache
    //   空闲时 dCache 优先
    //   释放: cache 的 rready 下降沿 (离开 S_BURST_RD)
    //         写 burst: bvalid && bready
    // ============================================================
    reg [1:0] owner;

    wire d_req = d_arvalid || d_awvalid;
    wire i_req = i_arvalid;

    // rready 边沿检测 (burst 完成时 cache rready 从 1→0)
    reg i_rready_d, d_rready_d;
    always @(posedge clk) begin
        i_rready_d <= i_rready;
        d_rready_d <= d_rready;
    end
    wire i_rready_fall = i_rready_d && !i_rready;   // iCache burst done
    wire d_rready_fall = d_rready_d && !d_rready;   // dCache read burst done

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            owner <= 2'b00;
        end else begin
            case (owner)
                2'b00: begin
                    if (d_req && ((d_arvalid && arready) || (d_awvalid && awready)))
                        owner <= 2'b10;
                    else if (i_req && i_arvalid && arready)
                        owner <= 2'b01;
                end
                2'b01: if (i_rready_fall) owner <= 2'b00;
                2'b10: if (d_rready_fall || (bvalid && bready)) owner <= 2'b00;
                default: owner <= 2'b00;
            endcase
        end
    end

    wire grant_d = (owner == 2'b10) || ((owner == 2'b00) && d_req);
    wire grant_i = (owner == 2'b01) || ((owner == 2'b00) && i_req && !d_req);

    // ============================================================
    // AR 通道 MUX
    // ============================================================
    assign arid    = grant_d ? ID_DACCESS : ID_IFETCH;
    assign araddr  = grant_d ? d_araddr   : i_araddr;
    assign arlen   = grant_d ? d_arlen    : i_arlen;
    assign arsize  = grant_d ? d_arsize   : i_arsize;
    assign arvalid = grant_d ? d_arvalid  : i_arvalid;
    assign arburst = 2'b01;
    assign arlock  = 2'b00;
    assign arcache = 4'b0000;
    assign arprot  = 3'b000;

    assign i_arready = arready && grant_i;
    assign d_arready = arready && grant_d;

    // ============================================================
    // R 通道 DEMUX
    // ============================================================
    assign i_rvalid = rvalid && (owner == 2'b01);
    assign i_rdata  = rdata;
    assign i_rresp  = rresp;
    assign i_rlast  = rlast;
    assign d_rvalid = rvalid && (owner == 2'b10);
    assign d_rdata  = rdata;
    assign d_rresp  = rresp;
    assign d_rlast  = rlast;
    assign rready = (owner == 2'b01) ? i_rready :
                    (owner == 2'b10) ? d_rready : 1'b0;

    // ============================================================
    // AW / W / B 通道 (仅 dCache)
    // ============================================================
    assign awid    = ID_DACCESS;
    assign awaddr  = d_awaddr;
    assign awlen   = d_awlen;
    assign awsize  = d_awsize;
    assign awvalid = d_awvalid;
    assign awburst = 2'b01;
    assign awlock  = 2'b00;
    assign awcache = 4'b0000;
    assign awprot  = 3'b000;
    assign d_awready = awready;

    assign wid    = ID_DACCESS;
    assign wdata  = d_wdata;
    assign wstrb  = d_wstrb;
    assign wlast  = d_wlast;
    assign wvalid = d_wvalid;
    assign d_wready = wready;

    assign d_bvalid = bvalid;
    assign d_bresp  = bresp;
    assign d_bid    = bid;
    assign bready = d_bready;

endmodule
