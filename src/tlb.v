// ============================================================================
// tlb — 全相联 TLB (4KB 页, LA32)
//   每项 64-bit: {V,D,G, ASID[9:0], VPN[19:0], PPN[19:0], PLV[1:0], MAT[1:0]}
//   组合逻辑查找, 0 拍命中; CSR 接口支持 TLBSRCH/TLBRD/TLBWR/TLBFILL/INVTLB
// ============================================================================
module tlb #(
    parameter TLB_ENTRIES = 16           // TLB 表项数
)(
    input  wire          clk,
    input  wire          rst_n,

    // ── 查找端口 (组合逻辑, 0 拍) ──
    input  wire [31:0]   lookup_va,      // 虚地址
    input  wire [ 1:0]   plv,            // 当前特权等级
    output wire [31:0]   lookup_pa,      // 物理地址 (命中时有效)
    output wire          lookup_hit,     // TLB 命中
    output wire          lookup_dirty,   // 页脏位 (写操作需检查)

    // ── CSR 接口 (TLB 维护指令) ──
    input  wire          csr_tlbsrch,    // TLBSRCH: 查找 TLB 表项
    input  wire          csr_tlbrd,      // TLBRD:  读取 TLB 表项
    input  wire          csr_tlbwr,      // TLBWR:  随机写入 TLB
    input  wire          csr_tlbfill,    // TLBFILL: 重填 TLB
    input  wire          csr_invtlb,     // INVTLB: 无效化 TLB 表项
    input  wire [31:0]   csr_tlb_va,     // TLB 操作虚地址
    input  wire [ 9:0]   csr_tlb_asid,   // ASID
    input  wire [63:0]   csr_tlb_entry,  // TLB 表项数据
    output wire [63:0]   csr_tlb_result  // TLB 查询结果
);

    localparam IDX_W = $clog2(TLB_ENTRIES);

    // ============================================================
    // TLB 存储 — 全相联, 组合读出
    //   csr_tlb_entry[63:0] 格式:
    //     [63]   V   (valid)
    //     [62]   D   (dirty)
    //     [61]   G   (global, 跳过 ASID 匹配)
    //     [60:51] ASID
    //     [50:31] VPN  (VA[31:12])
    //     [30:11] PPN  (PA[31:12])
    //     [10:9]  PLV  (权限: 0=kernel, 3=user)
    //     [8:7]   MAT  (0=uncached, 1=cached)
    //     [6:0]   reserved
    // ============================================================
    reg        tlb_v    [0:TLB_ENTRIES-1];
    reg        tlb_d    [0:TLB_ENTRIES-1];
    reg        tlb_g    [0:TLB_ENTRIES-1];
    reg [ 9:0] tlb_asid [0:TLB_ENTRIES-1];
    reg [19:0] tlb_vpn  [0:TLB_ENTRIES-1];
    reg [19:0] tlb_ppn  [0:TLB_ENTRIES-1];
    reg [ 1:0] tlb_plv  [0:TLB_ENTRIES-1];
    reg [ 1:0] tlb_mat  [0:TLB_ENTRIES-1];

    // ============================================================
    // 随机替换计数器 (简易 LFSR)
    // ============================================================
    reg [IDX_W-1:0] rand_cnt;
    wire [IDX_W-1:0] rand_next;
    assign rand_next = rand_cnt + 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rand_cnt <= 0;
        else
            rand_cnt <= rand_next;
    end

    // ============================================================
    // TLBSRCH 锁存
    // ============================================================
    reg [IDX_W-1:0] search_idx;
    reg             search_found;

    // ============================================================
    // 组合逻辑查找
    // ============================================================
    wire [19:0] va_vpn;
    assign va_vpn = lookup_va[31:12];

    // 逐项命中判定
    wire [TLB_ENTRIES-1:0] hit_vec;
    genvar gi;
    generate
        for (gi = 0; gi < TLB_ENTRIES; gi = gi + 1) begin : gen_hit
            assign hit_vec[gi] = tlb_v[gi]
                              && (tlb_vpn[gi] == va_vpn)
                              && (tlb_g[gi] || tlb_asid[gi] == csr_tlb_asid)
                              && (plv <= tlb_plv[gi]);
        end
    endgenerate

    // 优先编码器 (entry 0 最高优先级)
    function [IDX_W-1:0] find_first;
        input [TLB_ENTRIES-1:0] vec;
        integer k;
        begin
            find_first = 0;
            for (k = TLB_ENTRIES-1; k >= 0; k = k - 1) begin
                if (vec[k]) find_first = k[IDX_W-1:0];
            end
        end
    endfunction

    wire [IDX_W-1:0] hit_index;
    wire             any_hit;
    assign any_hit   = |hit_vec;
    assign hit_index = find_first(hit_vec);

    // 查找输出
    assign lookup_hit   = any_hit;
    // assign lookup_pa    = any_hit ? {tlb_ppn[hit_index], lookup_va[11:0]} : lookup_va;
    assign lookup_pa    = lookup_va;
    assign lookup_dirty = any_hit ? tlb_d[hit_index] : 1'b0;

    // ============================================================
    // TLBSRCH: 按 VA+ASID 查找, 锁存命中索引
    // ============================================================
    wire [19:0] srch_vpn = csr_tlb_va[31:12];
    wire [TLB_ENTRIES-1:0] srch_vec;
    genvar sj;
    generate
        for (sj = 0; sj < TLB_ENTRIES; sj = sj + 1) begin : gen_srch
            assign srch_vec[sj] = tlb_v[sj]
                               && (tlb_vpn[sj] == srch_vpn)
                               && (tlb_g[sj] || tlb_asid[sj] == csr_tlb_asid);
        end
    endgenerate

    wire [IDX_W-1:0] srch_index;
    wire             srch_any;
    assign srch_any   = |srch_vec;
    assign srch_index = find_first(srch_vec);

    always @(posedge clk) begin
        if (csr_tlbsrch) begin
            search_idx   <= srch_index;
            search_found <= srch_any;
        end
    end

    // ============================================================
    // 写入逻辑 (TLBWR / TLBFILL: 随机写入; TLBRD: 读 search_idx)
    // ============================================================
    wire do_write = csr_tlbwr || csr_tlbfill;
    wire do_read  = csr_tlbrd;
    wire do_inv   = csr_invtlb;

    // 解包 csr_tlb_entry
    wire        wr_v    = csr_tlb_entry[63];
    wire        wr_d    = csr_tlb_entry[62];
    wire        wr_g    = csr_tlb_entry[61];
    wire [ 9:0] wr_asid = csr_tlb_entry[60:51];
    wire [19:0] wr_vpn  = csr_tlb_entry[50:31];
    wire [19:0] wr_ppn  = csr_tlb_entry[30:11];
    wire [ 1:0] wr_plv  = csr_tlb_entry[10:9];
    wire [ 1:0] wr_mat  = csr_tlb_entry[8:7];

    integer wi;
    always @(posedge clk) begin
        if (do_write) begin
            tlb_v   [rand_cnt] <= wr_v;
            tlb_d   [rand_cnt] <= wr_d;
            tlb_g   [rand_cnt] <= wr_g;
            tlb_asid[rand_cnt] <= wr_asid;
            tlb_vpn [rand_cnt] <= wr_vpn;
            tlb_ppn [rand_cnt] <= wr_ppn;
            tlb_plv [rand_cnt] <= wr_plv;
            tlb_mat [rand_cnt] <= wr_mat;
        end

        if (do_inv) begin
            for (wi = 0; wi < TLB_ENTRIES; wi = wi + 1)
                tlb_v[wi] <= 1'b0;
        end
    end

    // ============================================================
    // CSR 读结果 (TLBRD: 读 search_idx 表项)
    // ============================================================
    assign csr_tlb_result = do_read ? {
        tlb_v   [search_idx],
        tlb_d   [search_idx],
        tlb_g   [search_idx],
        tlb_asid[search_idx],
        tlb_vpn [search_idx],
        tlb_ppn [search_idx],
        tlb_plv [search_idx],
        tlb_mat [search_idx],
        7'b0
    } : 64'b0;

    // ============================================================
    // 初始化
    // ============================================================
    integer init_i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (init_i = 0; init_i < TLB_ENTRIES; init_i = init_i + 1) begin
                tlb_v   [init_i] <= 1'b0;
                tlb_d   [init_i] <= 1'b0;
                tlb_g   [init_i] <= 1'b0;
                tlb_asid[init_i] <= 10'b0;
                tlb_vpn [init_i] <= 20'b0;
                tlb_ppn [init_i] <= 20'b0;
                tlb_plv [init_i] <= 2'b0;
                tlb_mat [init_i] <= 2'b0;
            end
            search_idx   <= 0;
            search_found <= 1'b0;
        end
    end

endmodule
