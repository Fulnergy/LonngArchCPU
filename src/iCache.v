// ============================================================================
// iCache — 2路组相联指令缓存 (64b 双发射取指, 只读)
// ============================================================================
// 特性:
//   - 2路组相联, NUM_SETS*2 行 (默认 128组×2路 = 256行)
//   - Cache line: LINE_SIZE_BYTES (默认 64B = 8 words × 64b)
//   - Data: 64b BRAM, 直接输出 dual_inst 指令对
//   - Fill: 2次 32b AXI 读 → 拼装 1个 64b BRAM 字 (half_buf)
//   - 替换策略: invalid 优先 + LRU (2路1bit伪LRU)
//   - Tag: distributed RAM (异步读, 0拍组合判定 hit/miss)
//   - Data: block RAM (同步读, 1拍)
//   - 外部存储侧: ext_ready 握手 (ext_we 恒为0, 仅读)
//
// ── 与 dCache 的区别 ──
//   1. 无 cpu_we / cpu_wdata / cpu_wstrb (只读)
//   2. Tag 无 dirty 位 (TAG_ENTRY_WIDTH = 1 + TAG_WIDTH)
//   3. 无 S_HIT_WR 状态 (无 RMW)
//   4. 无逐出写回 (evict 直接丢弃, dirty 永为0)
//   5. 无 write-allocate, 无 write miss
//
// ── Stall 协议 ──
//   读命中: 0 stall (S_IDLE stall=0 → S_DATA 交付)
//   读缺失: fill 全程 stall=1 (填充 ~35 拍)
//   背靠背读: 0 bubble (S_DATA→S_DATA, 每拍 1 个)
// ============================================================================

module iCache #(
    parameter ADDR_WIDTH       = 32,
    parameter DATA_WIDTH       = 64,         // 64b 双发射取指
    parameter LINE_SIZE_BYTES  = 64,
    parameter NUM_SETS         = 128,
    parameter NUM_WAYS         = 2
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ── CPU 侧 (只读) ──
    input  wire                     cpu_req,
    input  wire [ADDR_WIDTH-1:0]    cpu_addr,
    output reg  [DATA_WIDTH-1:0]    cpu_rdata,
    output wire                     cpu_stall,

    // ── 外部存储侧 (AXI 读通道, burst) ──
    output reg                      i_arvalid,
    output reg  [ADDR_WIDTH-1:0]    i_araddr,
    output reg  [ 7:0]              i_arlen,
    output reg  [ 2:0]              i_arsize,
    output reg  [ 1:0]              i_arburst,
    input  wire                     i_arready,

    input  wire                     i_rvalid,
    input  wire [31:0]              i_rdata,
    input  wire                     i_rlast,
    input  wire [ 1:0]              i_rresp,
    output reg                      i_rready
);

    // ============================================================
    // 派生参数
    // ============================================================
    localparam LINE_OFFSET_WIDTH = $clog2(LINE_SIZE_BYTES);
    localparam WORDS_PER_LINE    = LINE_SIZE_BYTES / (DATA_WIDTH / 8);
    localparam WORD_OFFSET_WIDTH = $clog2(WORDS_PER_LINE);
    localparam SET_INDEX_WIDTH   = $clog2(NUM_SETS);
    localparam TAG_WIDTH         = ADDR_WIDTH - SET_INDEX_WIDTH - LINE_OFFSET_WIDTH;
    localparam TAG_ENTRY_WIDTH   = 1 + TAG_WIDTH;          // 仅 valid + tag (无 dirty)
    localparam DATA_DEPTH        = NUM_SETS * WORDS_PER_LINE;
    localparam DATA_ADDR_WIDTH   = SET_INDEX_WIDTH + WORD_OFFSET_WIDTH;

    // ============================================================
    // 状态机
    // ============================================================
    reg [2:0] state, next_state;

    localparam S_IDLE        = 3'd0;
    localparam S_DATA        = 3'd1;
    localparam S_FILL_BURST  = 3'd2;    // 发送 AR burst, 等 arready
    localparam S_FILL_WAIT_R = 3'd3;    // 收集 R beats, 流式写 BRAM
    localparam S_RETRY       = 3'd4;

    // ============================================================
    // 请求锁存
    // ============================================================
    reg [ADDR_WIDTH-1:0] req_addr;

    // ============================================================
    // 地址选择: S_IDLE/S_DATA+cpu_req → cpu_addr, 其余 → req_addr
    // ============================================================
    wire is_req_taken;
    assign is_req_taken = ((state == S_IDLE) || (state == S_DATA)) && cpu_req;

    wire [ADDR_WIDTH-1:0] active_addr;
    assign active_addr = is_req_taken ? cpu_addr : req_addr;

    wire [SET_INDEX_WIDTH-1:0]   active_set;
    wire [WORD_OFFSET_WIDTH-1:0] active_word_off;
    wire [TAG_WIDTH-1:0]         active_tag;

    assign active_set      = active_addr[LINE_OFFSET_WIDTH +: SET_INDEX_WIDTH];
    assign active_word_off = active_addr[LINE_OFFSET_WIDTH-1:3];
    assign active_tag      = active_addr[ADDR_WIDTH-1 : LINE_OFFSET_WIDTH + SET_INDEX_WIDTH];

    wire [TAG_WIDTH-1:0]         req_tag;
    wire [SET_INDEX_WIDTH-1:0]   req_set;
    assign req_set = req_addr[LINE_OFFSET_WIDTH +: SET_INDEX_WIDTH];
    assign req_tag = req_addr[ADDR_WIDTH-1 : LINE_OFFSET_WIDTH + SET_INDEX_WIDTH];

    // ============================================================
    // Tag RAM — distributed RAM (异步读, 同步写)
    //   TAG_ENTRY: {valid, tag}
    // ============================================================
    (* ram_style = "distributed" *) reg [TAG_ENTRY_WIDTH-1:0] tag_ram0 [0:NUM_SETS-1];
    (* ram_style = "distributed" *) reg [TAG_ENTRY_WIDTH-1:0] tag_ram1 [0:NUM_SETS-1];

    reg                        tag_wr_en;
    reg                        tag_wr_way;
    reg  [SET_INDEX_WIDTH-1:0] tag_wr_addr;
    reg  [TAG_ENTRY_WIDTH-1:0] tag_wr_data;
    reg  [SET_INDEX_WIDTH-1:0] tag_rd_addr;

    wire [TAG_ENTRY_WIDTH-1:0] tag_async0, tag_async1;
    assign tag_async0 = tag_ram0[tag_rd_addr];
    assign tag_async1 = tag_ram1[tag_rd_addr];

    always @(posedge clk) begin
        if (tag_wr_en) begin
            if (tag_wr_way)
                tag_ram1[tag_wr_addr] <= tag_wr_data;
            else
                tag_ram0[tag_wr_addr] <= tag_wr_data;
        end
    end

    wire                tag_valid0;
    wire [TAG_WIDTH-1:0] tag_val0;
    wire                tag_valid1;
    wire [TAG_WIDTH-1:0] tag_val1;

    assign {tag_valid0, tag_val0} = tag_async0;
    assign {tag_valid1, tag_val1} = tag_async1;

    // ============================================================
    // 命中检测 — 纯组合逻辑
    // ============================================================
    wire [TAG_WIDTH-1:0] cmp_tag;
    assign cmp_tag = is_req_taken ? active_tag : req_tag;

    wire hit0, hit1;
    assign hit0 = tag_valid0 && (tag_val0 == cmp_tag);
    assign hit1 = tag_valid1 && (tag_val1 == cmp_tag);

    wire hit_now;
    assign hit_now = hit0 || hit1;

    // 锁存命中路 (供 S_DATA cpu_rdata 选择)
    reg hit1_latched;
    always @(posedge clk) begin
        if ((is_req_taken || state == S_RETRY) && hit_now)
            hit1_latched <= hit1;
    end

    // ============================================================
    // Data BRAM — block RAM (同步读, 1拍)
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

    // ============================================================
    // 请求锁存 @posedge
    // ============================================================
    always @(posedge clk) begin
        if (is_req_taken)
            req_addr <= cpu_addr;
    end

    // ============================================================
    // Victim 选择 — 组合逻辑
    // ============================================================
    wire                victim_way_comb;
    wire [TAG_WIDTH-1:0] victim_tag_comb;

    assign victim_way_comb = !tag_valid0       ? 1'b0 :
                              !tag_valid1       ? 1'b1 :
                              lru[active_set];

    assign victim_tag_comb = !tag_valid0       ? tag_val0 :
                              !tag_valid1       ? tag_val1 :
                              lru[active_set] ? tag_val1 : tag_val0;

    reg                victim_way;
    reg [TAG_WIDTH-1:0] victim_tag;

    wire miss_now;
    assign miss_now = (is_req_taken || state == S_RETRY) && !hit_now;

    always @(posedge clk) begin
        if (miss_now) begin
            victim_way <= victim_way_comb;
            victim_tag <= victim_tag_comb;
        end
    end

    // ============================================================
    // 字计数器 (64b BRAM word, 0-7)
    //   Burst R channel 流式接收, half_buf 拼装 64b 直接写 BRAM
    // ============================================================
    reg [WORD_OFFSET_WIDTH-1:0] word_cnt;   // 3b, 0-7
    reg                         half_lo;    // 0=等待低32b, 1=低32b已锁存等高位
    reg [31:0]                  half_buf;   // 低32b暂存

    wire word_cnt_last;
    assign word_cnt_last = (word_cnt == WORDS_PER_LINE - 1);

    always @(posedge clk) begin
        if (!rst_n) begin
            word_cnt <= 0;
            half_lo  <= 1'b0;
            half_buf <= 32'b0;
        end else begin
            case (state)
                S_FILL_WAIT_R: begin
                    if (i_rvalid && i_rready) begin
                        if (!half_lo) begin
                            half_buf <= i_rdata;
                            half_lo  <= 1'b1;
                        end else begin
                            half_lo <= 1'b0;
                            if (!word_cnt_last)
                                word_cnt <= word_cnt + 1;
                        end
                    end
                end
                S_FILL_BURST: ;              // 保持, 不重置
                default: begin
                    word_cnt <= 0;
                    half_lo  <= 1'b0;
                end
            endcase
        end
    end

    // ============================================================
    // 地址计算
    // ============================================================
    wire [ADDR_WIDTH-1:0] req_line_base;
    assign req_line_base = {req_tag, req_set, {LINE_OFFSET_WIDTH{1'b0}}};

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
            S_FILL_WAIT_R: begin
                data_addr  = {req_set, word_cnt};
                if (i_rvalid && i_rready && half_lo) begin
                    // 拼装 64b: {刚到达的高32b, 之前锁存的低32b}
                    data_wr_en   = 1'b1;
                    data_wr_way  = victim_way;
                    data_wr_data = {i_rdata, half_buf};

                    if (word_cnt_last) begin
                        tag_wr_en   = 1'b1;
                        tag_wr_way  = victim_way;
                        tag_wr_addr = req_set;
                        tag_wr_data = {1'b1, req_tag};
                    end
                end
            end

            default: ;
        endcase
    end

    // ============================================================
    // LRU 更新
    // ============================================================
    always @(posedge clk) begin
        if ((is_req_taken || state == S_RETRY) && hit_now) begin
            lru[active_set] <= hit0;
        end else if (state == S_FILL_WAIT_R && i_rvalid && i_rready && half_lo && word_cnt_last) begin
            lru[req_set] <= ~victim_way;
        end
    end

    // ============================================================
    // AXI 读通道接口 (组合逻辑)
    // ============================================================
    always @(*) begin
        // AR channel defaults
        i_arvalid = 1'b0;
        i_araddr  = req_line_base;
        i_arlen   = 8'd15;
        i_arsize  = 3'b010;
        i_arburst = 2'b01;

        // R channel default
        i_rready  = 1'b0;

        case (state)
            S_FILL_BURST: begin
                i_arvalid = 1'b1;
            end

            S_FILL_WAIT_R: begin
                i_rready = 1'b1;
            end

            default: ;
        endcase
    end

    // ============================================================
    // 状态转移 (burst fill: 一次 AR + 16-beat R)
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
            // ── S_IDLE: tag 异步读已 combo 出 hit/miss ──
            S_IDLE: begin
                if (cpu_req) begin
                    if (hit_now)
                        next_state = S_DATA;          // 命中 → 等 data BRAM
                    else
                        next_state = S_FILL_BURST;    // 缺失 → 发送 AR burst
                end
            end

            // ── S_DATA: data BRAM 输出到达 ──
            S_DATA: begin
                if (cpu_req) begin
                    if (hit_now)
                        next_state = S_DATA;          // 背靠背读命中
                    else
                        next_state = S_FILL_BURST;    // 背靠背缺失
                end else begin
                    next_state = S_IDLE;
                end
            end

            // ── 填充 ──
            S_FILL_BURST: begin
                if (i_arready)
                    next_state = S_FILL_WAIT_R;       // AR 握手 → 等 R burst
            end

            S_FILL_WAIT_R: begin
                if (i_rvalid && i_rready && i_rlast)
                    next_state = S_RETRY;              // R burst 完成
            end

            // ── S_RETRY: tag 刚刚有效, 重读 BRAM ──
            S_RETRY: begin
                next_state = S_DATA;
            end

            default: next_state = S_IDLE;
        endcase
    end

    // ============================================================
    // CPU 读数据 (组合逻辑)
    // ============================================================
    wire read_data_valid;
    assign read_data_valid = (state == S_DATA);

    always @(*) begin
        if (read_data_valid)
            cpu_rdata = hit1_latched ? data_bram1_dout : data_bram0_dout;
        else
            cpu_rdata = {DATA_WIDTH{1'b0}};
    end

    // ============================================================
    // CPU Stall (组合逻辑)
    //   stall=0: S_IDLE+!cpu_req, S_IDLE+hit, S_DATA(!sdata_stall)
    //   stall=1: miss, FILL, RETRY
    //   组合环路已通过 IF_Stage 的 pc_fetch 寄存器打破,
    //   cpu_stall 改为纯组合逻辑以消除 1 拍延迟,
    //   避免 miss 首拍 stall=0 导致 PC 抢跑跳过指令.
    // ============================================================
    wire sdata_stall;
    assign sdata_stall = (state == S_DATA) && cpu_req && !hit_now;

    assign cpu_stall = !((state == S_IDLE && !cpu_req) ||
                          (state == S_IDLE && cpu_req && hit_now) ||
                          (state == S_DATA && !sdata_stall));

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
