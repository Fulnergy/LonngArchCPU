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
    input  wire         if_en,           // 取指使能 (受全局 stall 调控)
    input  wire [31:0]  pc,              // 字节地址 (32b)
    output wire [63:0]  dual_inst,       // {inst_hi, inst_lo}
    output wire         if_stall,        // IF 级 stall

    // ── 外部存储 (iCache, 接 axi_bridge) ──
    output wire         ext_req,
    output wire         ext_we,
    output wire [31:0]  ext_addr,
    output wire [31:0]  ext_wdata,
    output wire [3:0]   ext_wstrb,
    input  wire [31:0]  ext_rdata,
    input  wire         ext_ready
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
        .cpu_req    (if_en),            // 受全局 stall 调控
        .cpu_addr   ({pc[31:3], 3'b0}), // 8B 对齐
        .cpu_rdata  (dual_inst),
        .cpu_stall  (if_stall),
        .ext_req    (ext_req),
        .ext_we     (ext_we),
        .ext_addr   (ext_addr),
        .ext_wdata  (ext_wdata),
        .ext_wstrb  (ext_wstrb),
        .ext_rdata  (ext_rdata),
        .ext_ready  (ext_ready)
    );

endmodule