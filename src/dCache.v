// ============================================================================
// dCache — 2路组相联数据缓存 (Write-Back + Write-Allocate)
// ============================================================================
// 特性:
//   - 2路组相联, NUM_SETS*2 行 (默认 128组×2路 = 256行)
//   - Cache line: LINE_SIZE_BYTES (默认 64B = 16 words)
//   - 写策略: write-back + write-allocate
//   - 替换策略: invalid 优先 + LRU (2路1bit伪LRU)
//   - Tag: distributed RAM (异步读, 0拍组合判定 hit/miss)
//   - Data: block RAM (同步读, 1拍)
//   - 外部存储侧: ext_ready 握手
//
// ── Stall 协议 ──
//   cpu_stall 为组合逻辑输出, CPU 内部经 1 拍寄存器 (stall_reg) 后控制流水线:
//     stall=1 的周期 → 下一 posedge 流水线冻结 (寄存器自保持)
//     stall=0 的周期 → 下一 posedge 流水线正常推进
//
//   关键时序约束:
//     - S_IDLE + read  hit: 本拍 stall=0, 下一拍 data BRAM 输出到达 S_DATA,
//                           stall_reg 滞后 1 拍正好对齐 → 0 stall
//     - S_IDLE + write hit: 本拍 stall=1, BRAM 读等待, 下一拍 S_HIT_WR 做 RMW
//     - S_IDLE + miss:      本拍 stall=1, 进入 EVICT/FILL
//     - S_DATA:             data BRAM 输出有效, 本拍 stall=0 交付读数据
//     - S_DATA + read hit:  背靠背读, stall=0, S_DATA→S_DATA 无气泡
//     - S_DATA + write hit: 背靠背写, stall=1, S_DATA→S_HIT_WR, 堵住后续指令
//     - S_DATA + miss:      背靠背缺失, stall=1, S_DATA→EVICT/FILL
//     - S_HIT_WR:           RMW 提交, stall=0 释放流水线 (写指令离开 MEM)
//     - EVICT/FILL/RETRY:   始终 stall=1
//
//   延迟总结:
//     读命中:  0 stall (S_IDLE stall=0 → S_DATA 交付)
//     写命中:  1 stall (S_IDLE stall=1 + S_HIT_WR stall=1, RMW 幂等)
//     读缺失:  fill 全程 stall=1 (逐出脏行 + 填充, ~35~70 拍)
//     写缺失:  同读缺失 + S_HIT_WR
//     背靠背读: 0 bubble (S_DATA→S_DATA, 每拍 1 个读)
// ============================================================================

module dCache #(
    parameter ADDR_WIDTH       = 32,
    parameter DATA_WIDTH       = 32,
    parameter LINE_SIZE_BYTES  = 64,
    parameter NUM_SETS         = 128,
    parameter NUM_WAYS         = 2
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ============================================================
    // CPU 侧 — stall 协议
    // ============================================================
    input  wire                     cpu_req,
    input  wire                     cpu_we,
    input  wire [1:0]               cpu_size,
    input  wire [ADDR_WIDTH-1:0]    cpu_addr,
    input  wire [DATA_WIDTH-1:0]    cpu_wdata,
    input  wire [3:0]               cpu_wstrb,
    output reg  [DATA_WIDTH-1:0]    cpu_rdata,
    output wire                     cpu_stall,

    // ── AXI-R (读 burst, fill) ──
    output reg                      arvalid,
    output reg  [ADDR_WIDTH-1:0]    araddr,
    output reg  [ 7:0]              arlen,
    output reg  [ 2:0]              arsize,
    input  wire                     arready,
    input  wire                     rvalid,
    input  wire [DATA_WIDTH-1:0]    rdata,
    input  wire [ 1:0]              rresp,
    input  wire                     rlast,
    output reg                      rready,
    // ── AXI-W (写 burst, eviction) ──
    output reg                      awvalid,
    output reg  [ADDR_WIDTH-1:0]    awaddr,
    output reg  [ 7:0]              awlen,
    output reg  [ 2:0]              awsize,
    input  wire                     awready,
    output reg                      wvalid,
    output reg  [DATA_WIDTH-1:0]    wdata,
    output reg  [ 3:0]              wstrb,
    output reg                      wlast,
    input  wire                     wready,
    input  wire                     bvalid,
    input  wire [ 1:0]              bresp,
    input  wire [ 3:0]              bid,
    output reg                      bready
);

    // ============================================================
    // 派生参数
    // ============================================================
    localparam LINE_OFFSET_WIDTH = $clog2(LINE_SIZE_BYTES);
    localparam WORDS_PER_LINE    = LINE_SIZE_BYTES / (DATA_WIDTH / 8);
    localparam WORD_OFFSET_WIDTH = $clog2(WORDS_PER_LINE);
    localparam SET_INDEX_WIDTH   = $clog2(NUM_SETS);
    localparam TAG_WIDTH         = ADDR_WIDTH - SET_INDEX_WIDTH - LINE_OFFSET_WIDTH;
    localparam TAG_ENTRY_WIDTH   = 2 + TAG_WIDTH;
    localparam DATA_DEPTH        = NUM_SETS * WORDS_PER_LINE;
    localparam DATA_ADDR_WIDTH   = SET_INDEX_WIDTH + WORD_OFFSET_WIDTH;
    localparam CNT_WIDTH         = WORD_OFFSET_WIDTH + 1;

    // ============================================================
    // 状态机 (提前声明, 地址逻辑需要引用 state / S_*)
    // ============================================================
    reg [3:0] state, next_state;

    localparam S_IDLE       = 4'd0;
    localparam S_DATA       = 4'd1;
    localparam S_HIT_WR     = 4'd2;
    localparam S_EVICT_RD   = 4'd3;
    localparam S_AW_REQ     = 4'd4;
    localparam S_BURST_WR   = 4'd5;
    localparam S_AR_REQ     = 4'd6;
    localparam S_BURST_RD   = 4'd7;
    localparam S_RETRY      = 4'd8;
    localparam S_BRESP      = 4'd9;

    // ============================================================
    // 请求锁存寄存器 (提前声明, active_addr 逻辑需要引用 req_addr)
    // ============================================================
    reg [ADDR_WIDTH-1:0]  req_addr;
    reg [DATA_WIDTH-1:0]  req_wdata;
    reg [3:0]             req_wstrb;
    reg                   req_we;
    reg [1:0]             req_size;

    // ============================================================
    // 地址选择: S_IDLE/S_DATA+cpu_req → cpu_addr, 其余 → req_addr
    //   S_DATA 中接受新请求实现背靠背 hit 流水化
    // ============================================================
    wire is_req_taken;
    assign is_req_taken = ((state == S_IDLE) || (state == S_DATA)) && cpu_req;

    wire [ADDR_WIDTH-1:0] active_addr;
    assign active_addr = is_req_taken ? cpu_addr : req_addr;

    // 从 active_addr 提取字段 (组合)
    wire [SET_INDEX_WIDTH-1:0]   active_set;
    wire [WORD_OFFSET_WIDTH-1:0] active_word_off;
    wire [TAG_WIDTH-1:0]         active_tag;

    assign active_set      = active_addr[LINE_OFFSET_WIDTH +: SET_INDEX_WIDTH];
    assign active_word_off = active_addr[LINE_OFFSET_WIDTH-1:2];
    assign active_tag      = active_addr[ADDR_WIDTH-1 : LINE_OFFSET_WIDTH + SET_INDEX_WIDTH];

    // 从 req_addr 提取 tag (用于 tag 比较, S_IDLE_req 末锁存后与 cpu_addr 一致)
    wire [TAG_WIDTH-1:0] req_tag;
    wire [SET_INDEX_WIDTH-1:0] req_set;
    assign req_set = req_addr[LINE_OFFSET_WIDTH +: SET_INDEX_WIDTH];
    assign req_tag = req_addr[ADDR_WIDTH-1 : LINE_OFFSET_WIDTH + SET_INDEX_WIDTH];

    // ============================================================
    // Tag RAM — distributed RAM (异步读, 0 拍; 同步写)
    //   读路径: tag_ram[tag_addr] → 组合逻辑 → hit0/hit1 → cpu_stall
    // ============================================================
    (* ram_style = "distributed" *) reg [TAG_ENTRY_WIDTH-1:0] tag_ram0 [0:NUM_SETS-1];
    (* ram_style = "distributed" *) reg [TAG_ENTRY_WIDTH-1:0] tag_ram1 [0:NUM_SETS-1];

    reg                        tag_wr_en;
    reg                        tag_wr_way;
    reg  [SET_INDEX_WIDTH-1:0] tag_wr_addr;
    reg  [TAG_ENTRY_WIDTH-1:0] tag_wr_data;

    reg  [SET_INDEX_WIDTH-1:0] tag_rd_addr;

    // 异步读 (组合逻辑)
    wire [TAG_ENTRY_WIDTH-1:0] tag_async0, tag_async1;
    assign tag_async0 = tag_ram0[tag_rd_addr];
    assign tag_async1 = tag_ram1[tag_rd_addr];

    // 同步写
    always @(posedge clk) begin
        if (tag_wr_en) begin
            if (tag_wr_way)
                tag_ram1[tag_wr_addr] <= tag_wr_data;
            else
                tag_ram0[tag_wr_addr] <= tag_wr_data;
        end
    end

    wire                tag_valid0, tag_dirty0;
    wire [TAG_WIDTH-1:0] tag_val0;
    wire                tag_valid1, tag_dirty1;
    wire [TAG_WIDTH-1:0] tag_val1;

    assign {tag_valid0, tag_dirty0, tag_val0} = tag_async0;
    assign {tag_valid1, tag_dirty1, tag_val1} = tag_async1;

    // ============================================================
    // 命中检测 — 纯组合逻辑 (tag 异步读, 0 拍)
    //   cmp_tag: S_IDLE_req → active_tag (=cpu_addr tag), 其余 → req_tag
    //   这确保 S_IDLE 拍的 tag 比较用当前请求的 tag 而非旧 req_tag
    // ============================================================
    wire [TAG_WIDTH-1:0] cmp_tag;
    assign cmp_tag = is_req_taken ? active_tag : req_tag;

    wire hit0, hit1;
    assign hit0 = tag_valid0 && (tag_val0 == cmp_tag);
    assign hit1 = tag_valid1 && (tag_val1 == cmp_tag);

    // 锁存命中路 (S_IDLE_req 或 S_RETRY 末锁存, 供 S_HIT_WR RMW 使用)
    reg hit1_latched;

    always @(posedge clk) begin
        if ((is_req_taken || state == S_RETRY) && (hit0 || hit1))
            hit1_latched <= hit1;
    end

    // ============================================================
    // Data BRAM — block RAM (同步读, 1 拍)
    // ============================================================
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] data_bram0 [0:DATA_DEPTH-1];
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] data_bram1 [0:DATA_DEPTH-1];
    reg [DATA_WIDTH-1:0] data_bram0_dout, data_bram1_dout;

    reg                        data_wr_en;
    reg                        data_wr_way;
    reg  [DATA_ADDR_WIDTH-1:0] data_addr;
    reg  [DATA_WIDTH-1:0]      data_wr_data;

    always @(posedge clk) begin
        data_bram0_dout <= data_bram0[data_addr];
        data_bram1_dout <= data_bram1[data_addr];
        if (data_wr_en) begin
            if (data_wr_way)
                data_bram1[data_addr] <= data_wr_data;
            else
                data_bram0[data_addr] <= data_wr_data;
        end
    end

    // ============================================================
    // LRU: 0=way0是LRU, 1=way1是LRU
    // ============================================================
    reg [NUM_SETS-1:0] lru;

    always @(posedge clk) begin
        if (is_req_taken) begin
            req_addr  <= cpu_addr;
            req_wdata <= cpu_wdata;
            req_wstrb <= cpu_wstrb;
            req_we    <= cpu_we;
            req_size  <= cpu_size;
        end
    end

    // ============================================================
    // Victim 选择 — 组合逻辑 (S_IDLE miss 或 S_DATA miss 时有效)
    //   使用 active_set (在 S_IDLE 时 = cpu_set, 在 S_DATA 时 = req_set)
    // ============================================================
    wire                victim_way_comb;
    wire                victim_dirty_comb;
    wire [TAG_WIDTH-1:0] victim_tag_comb;

    assign victim_way_comb = !tag_valid0       ? 1'b0 :
                              !tag_valid1       ? 1'b1 :
                              lru[active_set];

    assign victim_dirty_comb = !tag_valid0       ? 1'b0 :
                                !tag_valid1       ? 1'b0 :
                                lru[active_set] ? tag_dirty1 : tag_dirty0;

    assign victim_tag_comb = !tag_valid0       ? tag_val0 :
                              !tag_valid1       ? tag_val1 :
                              lru[active_set] ? tag_val1 : tag_val0;

    reg                victim_way;
    reg                victim_dirty;
    reg [TAG_WIDTH-1:0] victim_tag;

    // 在 miss 发生的拍末锁存 (S_IDLE miss 或 S_RETRY miss)
    wire miss_now;
    assign miss_now = (is_req_taken || state == S_RETRY) && !hit0 && !hit1;

    always @(posedge clk) begin
        if (miss_now) begin
            victim_way   <= victim_way_comb;
            victim_dirty <= victim_dirty_comb;
            victim_tag   <= victim_tag_comb;
        end
    end

    // ============================================================
    // 字计数器 (burst 适配)
    // ============================================================
    reg [CNT_WIDTH-1:0] word_cnt;
    reg [DATA_WIDTH-1:0] rdata_latched;
    reg [DATA_WIDTH-1:0] evict_data_reg;
    reg                  evict_rd_cyc;   // S_EVICT_RD 第2拍标志
    wire word_cnt_last;
    assign word_cnt_last = (word_cnt == WORDS_PER_LINE - 1);
    wire burst_rd_done;
    assign burst_rd_done = rvalid && rready && rlast;

    always @(posedge clk) begin
        if (!rst_n) begin
            rdata_latched <= 32'b0;
        end else if (rvalid && rready) begin
            rdata_latched <= rdata;
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            word_cnt <= 0;
            evict_rd_cyc <= 1'b0;
        end else begin
            case (state)
                S_BURST_RD: if (rvalid && rready) word_cnt <= word_cnt + 1;
                S_BURST_WR: if (wvalid && wready) word_cnt <= word_cnt + 1;
                default:    word_cnt <= 0;
            endcase
            // S_EVICT_RD 第2拍标志
            if (state == S_EVICT_RD)
                evict_rd_cyc <= 1'b1;
            else
                evict_rd_cyc <= 1'b0;
        end
    end

    // evict_data 在 S_EVICT_RD 第2拍锁存 (BRAM 已输出 word[0]),
    // S_BURST_WR 逐拍预读更新 (data_addr = word_cnt+1 → BRAM 下一拍输出下一字)
    always @(posedge clk) begin
        if ((state == S_EVICT_RD && evict_rd_cyc) || state == S_BURST_WR)
            evict_data_reg <= evict_data;
    end

    // ============================================================
    // 行基地址 (用于外部访存)
    // ============================================================
    wire [ADDR_WIDTH-1:0] req_line_base;
    wire [ADDR_WIDTH-1:0] victim_line_base;
    wire [ADDR_WIDTH-1:0] word_addr_offset;

    assign req_line_base    = {req_tag,    req_set, {LINE_OFFSET_WIDTH{1'b0}}};
    assign victim_line_base = {victim_tag, req_set, {LINE_OFFSET_WIDTH{1'b0}}};
    assign word_addr_offset = {word_cnt[WORD_OFFSET_WIDTH-1:0],
                               {($clog2(DATA_WIDTH/8)){1'b0}}};

    // ============================================================
    // RMW 合并
    // ============================================================
    wire [DATA_WIDTH-1:0] hit_data_word;
    assign hit_data_word = hit1_latched ? data_bram1_dout : data_bram0_dout;

    wire [DATA_WIDTH-1:0] merged_wdata;
    genvar gb;
    generate
        for (gb = 0; gb < DATA_WIDTH / 8; gb = gb + 1) begin : gen_byte_merge
            assign merged_wdata[gb*8 +: 8] = req_wstrb[gb]
                                            ? req_wdata[gb*8 +: 8]
                                            : hit_data_word[gb*8 +: 8];
        end
    endgenerate

    wire [DATA_WIDTH-1:0] evict_data;
    assign evict_data = victim_way ? data_bram1_dout : data_bram0_dout;

    // ============================================================
    // BRAM 地址 & 写控制 (组合逻辑)
    // ============================================================
    always @(*) begin
        tag_rd_addr = active_set;
        tag_wr_en   = 1'b0;
        tag_wr_way  = 1'b0;
        tag_wr_addr = req_set;
        tag_wr_data = 0;

        data_addr    = {active_set, active_word_off};
        data_wr_en   = 1'b0;
        data_wr_way  = 1'b0;
        data_wr_data = 0;

        case (state)
            S_EVICT_RD: begin
                data_addr = {req_set, word_cnt[WORD_OFFSET_WIDTH-1:0]};
            end

            S_BURST_WR: begin
                // 预读下一个 BRAM word, 供下一拍 wdata 使用
                data_addr = {req_set, word_cnt[WORD_OFFSET_WIDTH-1:0] + 1'b1};
            end

            S_HIT_WR: begin
                data_addr    = {req_set, active_word_off};
                data_wr_en   = 1'b1;
                data_wr_way  = hit1_latched;
                data_wr_data = merged_wdata;
                tag_wr_en   = 1'b1;
                tag_wr_way  = hit1_latched;
                tag_wr_addr = req_set;
                tag_wr_data = {1'b1, 1'b1, req_tag};
            end

            S_BURST_RD: begin
                data_addr    = {req_set, word_cnt[WORD_OFFSET_WIDTH-1:0]};
                data_wr_en   = rvalid && rready;
                data_wr_way  = victim_way;
                data_wr_data = rdata;
                if (rvalid && rready && word_cnt_last) begin
                    tag_wr_en   = 1'b1;
                    tag_wr_way  = victim_way;
                    tag_wr_addr = req_set;
                    tag_wr_data = {1'b1, 1'b0, req_tag};
                end
            end

            default: ;
        endcase
    end

    // ============================================================
    // LRU 更新
    // ============================================================
    always @(posedge clk) begin
        if ((is_req_taken || state == S_RETRY) && (hit0 || hit1)) begin
            lru[active_set] <= hit0;
        end else if (state == S_BURST_RD && rvalid && rready && word_cnt_last) begin
            lru[req_set] <= ~victim_way;
        end
    end

    // ============================================================
    // AXI 接口
    // ============================================================
    always @(*) begin
        arvalid = (state == S_AR_REQ);
        araddr  = req_line_base;
        arlen   = 8'd15;
        arsize  = 3'b010;
        rready  = (state == S_BURST_RD);

        awvalid = (state == S_AW_REQ);
        awaddr  = victim_line_base;
        awlen   = 8'd15;
        awsize  = 3'b010;

        wvalid  = (state == S_BURST_WR);
        wdata   = evict_data_reg;
        wstrb   = 4'b1111;
        wlast   = (state == S_BURST_WR) && word_cnt_last;

        bready  = (state == S_BURST_WR) || (state == S_BRESP);
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
                if (cpu_req) begin
                    if (hit0 || hit1) begin
                        if (cpu_we)
                            next_state = S_HIT_WR;
                        else
                            next_state = S_DATA;
                    end else begin
                        next_state = victim_dirty_comb ? S_EVICT_RD : S_AR_REQ;
                    end
                end
            end

            S_DATA: begin
                if (cpu_req) begin
                    if (hit0 || hit1) begin
                        if (cpu_we)
                            next_state = S_HIT_WR;
                        else
                            next_state = S_DATA;
                    end else begin
                        next_state = victim_dirty_comb ? S_EVICT_RD : S_AR_REQ;
                    end
                end else begin
                    next_state = S_IDLE;
                end
            end

            S_HIT_WR: begin
                next_state = S_IDLE;
            end

            S_EVICT_RD: begin
                // 第1拍: 设置 BRAM 地址 (data_addr = word[0]), 等待
                // 第2拍: BRAM 输出 word[0], evict_data_reg 锁存, 进入 S_AW_REQ
                if (evict_rd_cyc)
                    next_state = S_AW_REQ;
            end

            S_AW_REQ: begin
                if (awready)
                    next_state = S_BURST_WR;
            end

            S_BURST_WR: begin
                if (wvalid && wready && word_cnt_last)
                    next_state = S_BRESP;
            end

            S_BRESP: begin
                if (bvalid)
                    next_state = S_AR_REQ;
            end

            S_AR_REQ: begin
                if (arready)
                    next_state = S_BURST_RD;
            end

            S_BURST_RD: begin
                // 末 beat 写入时退出 (不依赖 rlast, 因其为寄存器滞后一拍)
                if (rvalid && rready && word_cnt_last)
                    next_state = S_RETRY;
            end

            S_RETRY: begin
                if (req_we)
                    next_state = S_HIT_WR;
                else
                    next_state = S_DATA;
            end

            default: next_state = S_IDLE;
        endcase
    end

    // ============================================================
    // CPU 读数据
    //   S_DATA: BRAM dout 有效, 组合输出
    //   其他:   保持 last_read_data (S_DATA 拍锁存), 防止 stall 期间归零
    // ============================================================
    wire read_data_valid;
    assign read_data_valid = (state == S_DATA);

    reg [DATA_WIDTH-1:0] last_read_data;

    always @(posedge clk) begin
        if (!rst_n)
            last_read_data <= {DATA_WIDTH{1'b0}};
        else if (state == S_DATA)
            last_read_data <= hit1_latched ? data_bram1_dout : data_bram0_dout;
    end

    always @(*) begin
        if (read_data_valid)
            cpu_rdata = hit1_latched ? data_bram1_dout : data_bram0_dout;
        else
            cpu_rdata = last_read_data;
    end

    // ============================================================
    // CPU Stall (组合逻辑)
    //
    // 协议: stall=1 的周期 → 下一 posedge stall_reg 锁存 1 → 流水线冻结
    //       stall=0 的周期 → 下一 posedge stall_reg 锁存 0 → 流水线推进
    //
    // stall=0 条件 (任一成立):
    //   S_IDLE && !cpu_req                空闲, 无请求
    //   S_IDLE && cpu_req && read_hit     读命中 0-stall (stall_reg 滞后对齐)
    //   S_DATA && !sdata_stall            data 交付 / 背靠背 read hit
    //   S_HIT_WR                          RMW 已完成 → 释放流水线
    //
    // stall=1 条件 (其他所有):
    //   S_IDLE && write_hit               BRAM 读等待 → S_HIT_WR
    //   S_IDLE && miss                    EVICT/FILL
    //   S_DATA && sdata_stall             write hit / miss 堵后续指令
    //   S_EVICT_RD/WR/DONE/FILL_REQ/WR/RETRY
    //
    // sdata_stall: S_DATA 拍有新请求且不是 read hit → 必须 stall
    //   - write hit: 下一拍 S_HIT_WR 不能接受新请求
    //   - miss:      下一拍 EVICT/FILL 不能接受新请求
    // ============================================================
    wire hit_now;
    assign hit_now = hit0 || hit1;

    wire sdata_stall;
    assign sdata_stall = (state == S_DATA) && cpu_req && !(hit_now && !cpu_we);

    assign cpu_stall = !((state == S_IDLE && !cpu_req) ||
                          (state == S_IDLE && cpu_req && hit_now && !cpu_we) ||
                          (state == S_DATA && !sdata_stall) ||
                          (state == S_HIT_WR));

    // ============================================================
    // 初始化
    // ============================================================
    integer init_i;
    initial begin
        for (init_i = 0; init_i < NUM_SETS; init_i = init_i + 1) begin
            tag_ram0[init_i] = 0;
            tag_ram1[init_i] = 0;
        end
        for (init_i = 0; init_i < DATA_DEPTH; init_i = init_i + 1) begin
            data_bram0[init_i] = 0;
            data_bram1[init_i] = 0;
        end
        lru = 0;
    end

endmodule
