// ============================================================================
// IF_Stage — 双发射取指, 64b iCache 直接输出 dual_inst
// ============================================================================
//   iCache 已改为 64b data, BRAM 存整条 inst pair
//   读命中: 0 stall, S_DATA 交付 64b
//   读缺失: stall=1, fill 完成后交付
// ============================================================================

module IF_Stage (
    input  wire         clk,
    input  wire         rst_n,

    // ── CPU 侧 ──
    input  wire [31:0]  pc,              // 字节地址 (32b)
    output wire [63:0]  dual_inst,       // {inst_hi, inst_lo}
    output wire         if_stall,        // IF 级 stall

    // ── AXI-R (iCache burst fill) ──
    output wire         arvalid,
    output wire [31:0]  araddr,
    output wire [ 7:0]  arlen,
    output wire [ 2:0]  arsize,
    input  wire         arready,

    input  wire         rvalid,
    input  wire [31:0]  rdata,
    input  wire [ 1:0]  rresp,
    input  wire         rlast,
    output wire         rready
);

    // ============================================================
    // iCache (64b data, 直接输出 dual_inst)
    // ============================================================
    iCache #(
        .ADDR_WIDTH      (32),
        .DATA_WIDTH      (64),
        .LINE_SIZE_BYTES (64),
        .NUM_SETS        (128),
        .NUM_WAYS        (2)
    ) u_icache (
        .clk        (clk),
        .rst_n      (rst_n),
        .cpu_req    (1'b1),             // 始终请求, cache 内部处理 busy
        .cpu_addr   ({pc[31:3], 3'b0}), // 8B 对齐
        .cpu_rdata  (dual_inst),
        .cpu_stall  (if_stall),
        .arvalid    (arvalid),
        .araddr     (araddr),
        .arlen      (arlen),
        .arsize     (arsize),
        .arready    (arready),
        .rvalid     (rvalid),
        .rdata      (rdata),
        .rresp      (rresp),
        .rlast      (rlast),
        .rready     (rready)
    );

endmodule