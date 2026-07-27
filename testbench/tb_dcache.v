`timescale 1ns / 1ps

// ============================================================================
// dCache 测试平台
//   覆盖: 读命中/缺失, 写命中/缺失, victim 替换 (LRU + invalid优先),
//         write-back 写回, write-allocate 写分配
//
// 地址选择: 全部映射到 set index = 0 (addr[12:6]=0)
//   不同 tag 值实现 set 内 conflict:
//     A0=32'h0000_0000  tag=0,  set=0
//     A1=32'h0000_2000  tag=1,  set=0   (1<<13)
//     A2=32'h0000_4000  tag=2,  set=0   (2<<13)
//     A3=32'h0000_6000  tag=3,  set=0   (3<<13)
//     A4=32'h0000_8000  tag=4,  set=0   (4<<13)
//     A5=32'h0000_A000  tag=5,  set=0   (5<<13)
//
// 外部存储: 每 line 16 words, word N 存 {tag[7:0], N[7:0]} 模式
//   例: tag=0 line → word0=32'h0000_0000, word1=32'h0000_0100, ...
//       tag=1 line → word0=32'h0100_0000, word1=32'h0100_0100, ...
// ============================================================================

module tb_dcache;

    parameter CLK_PERIOD = 10;           // 100MHz

    // ============================================================
    // DUT 信号
    // ============================================================
    reg         clk;
    reg         rst_n;

    reg         cpu_req;
    reg         cpu_we;
    reg  [1:0]  cpu_size;
    reg  [31:0] cpu_addr;
    reg  [31:0] cpu_wdata;
    reg  [3:0]  cpu_wstrb;
    wire [31:0] cpu_rdata;
    wire        cpu_stall;

    wire        ext_req;
    wire        ext_we;
    wire [31:0] ext_addr;
    wire [31:0] ext_wdata;
    wire [3:0]  ext_wstrb;
    wire [31:0] ext_rdata;
    reg         ext_ready;

    // ============================================================
    // DUT 例化
    // ============================================================
    dCache #(
        .ADDR_WIDTH      (32),
        .DATA_WIDTH      (32),
        .LINE_SIZE_BYTES (64),
        .NUM_SETS        (128),
        .NUM_WAYS        (2)
    ) u_dcache (
        .clk        (clk),
        .rst_n      (rst_n),
        .cpu_req    (cpu_req),
        .cpu_we     (cpu_we),
        .cpu_size   (cpu_size),
        .cpu_addr   (cpu_addr),
        .cpu_wdata  (cpu_wdata),
        .cpu_wstrb  (cpu_wstrb),
        .cpu_rdata  (cpu_rdata),
        .cpu_stall  (cpu_stall),
        .ext_req    (ext_req),
        .ext_we     (ext_we),
        .ext_addr   (ext_addr),
        .ext_wdata  (ext_wdata),
        .ext_wstrb  (ext_wstrb),
        .ext_rdata  (ext_rdata),
        .ext_ready  (ext_ready)
    );

    // ============================================================
    // 时钟 & 复位
    // ============================================================
    initial clk = 0;
    always #(CLK_PERIOD / 2) clk = ~clk;

    initial begin
        rst_n = 1'b0;
        #(CLK_PERIOD);
        rst_n = 1'b1;
    end

    // ============================================================
    // 外部存储模拟 (类SRAM, 固定 1 拍延迟)
    //   时序: ext_req=1 (C0) → ext_ready=1 + ext_rdata 有效 (C1)
    //   读: 当拍锁存 ext_addr, 下一拍驱动 ext_rdata
    //   写: 当拍锁存 ext_addr+ext_wdata, 下一拍写入 ext_mem
    // ============================================================
    localparam EXT_MEM_DEPTH = 16384;
    reg [31:0] ext_mem [0:EXT_MEM_DEPTH-1];
    parameter ADDR_MSB = 31;

    // 1 拍延迟寄存器
    reg         ext_req_q;
    reg         ext_we_q;
    reg  [31:0] ext_addr_q;
    reg  [31:0] ext_wdata_q;

    always @(posedge clk) begin
        ext_req_q  <= ext_req;
        ext_we_q   <= ext_we;
        ext_addr_q <= ext_addr;
        ext_wdata_q<= ext_wdata;
    end

    // ext_ready: ext_req 后 1 拍
    always @(posedge clk) begin
        if (!rst_n)
            ext_ready <= 1'b0;
        else
            ext_ready <= ext_req;
    end

    // ext_rdata + write: 基于上一拍的 ext_req_q
    reg [31:0] ext_rdata_r;
    always @(posedge clk) begin
        if (ext_req_q && !ext_we_q)
            ext_rdata_r <= ext_mem[ext_addr_q[ADDR_MSB:2]];
        if (ext_req_q && ext_we_q)
            ext_mem[ext_addr_q[ADDR_MSB:2]] <= ext_wdata_q;
    end

    assign ext_rdata = ext_rdata_r;

    // ============================================================
    // 外部存储初始化
    //   tag=N 的 line: word M = {8'd(N), 8'd(M), 8'd(M), 8'd(N)}
    //   例: tag=0 line → w0=32'h0000_0000, w1=32'h0001_0100, w2=32'h0002_0200...
    //   用于验证读数据正确性
    // ============================================================
    task init_ext_mem;
        integer tag, wn;
        integer line_base;
        begin
            for (tag = 0; tag < 8; tag = tag + 1) begin
                line_base = tag << 13;              // {tag, 7'b0, 6'b0}
                for (wn = 0; wn < 16; wn = wn + 1) begin
                    ext_mem[(line_base >> 2) + wn] = {tag[7:0], wn[7:0], wn[7:0], tag[7:0]};
                end
            end
            $display("[TB] ext_mem initialized: 8 lines (tags 0..7), 64B each");
        end
    endtask

    // ============================================================
    // 地址宏: 指定 tag, word offset, byte offset
    //   addr = {tag[18:0], 7'b0, 6'd0} + {14'b0, word_off[3:0], 2'b0} + byte_off
    // ============================================================
    function [31:0] make_addr;
        input [18:0] tag;
        input [3:0]  word_off;
        input [1:0]  byte_off;
        begin
            make_addr = {tag, 7'b0, 6'b0} | {14'b0, word_off, 2'b0} | {30'b0, byte_off};
        end
    endfunction

    // 期望的外部存储读数据: word N of tag T = {T[7:0], N[7:0], N[7:0], T[7:0]}
    function [31:0] expected_word;
        input [18:0] tag;
        input [3:0]  word_off;
        begin
            expected_word = {{4'b0, tag[3:0]}, {4'b0, word_off},
                             {4'b0, word_off}, {4'b0, tag[3:0]}};
        end
    endfunction

    // ============================================================
    // ============================================================
    // CPU 请求任务: 在时钟沿之前设置信号, 等待 stall=0 完成
    // ============================================================

    // ── 读操作 ──
    task do_read;
        input  [31:0] addr;
        input  [1:0]  size;
        output [31:0] data;
        integer wait_cnt;
        begin
            cpu_req   = 1'b1;
            cpu_we    = 1'b0;
            cpu_addr  = addr;
            cpu_size  = size;
            cpu_wdata = 32'bx;
            cpu_wstrb = 4'b0000;

            @(posedge clk);
            wait_cnt = 1;

            while (cpu_stall !== 1'b0) begin
                @(posedge clk);
                wait_cnt = wait_cnt + 1;
            end

            // 先撤 cpu_req, 防止 DUT 在 S_DATA 误判为背靠背请求
            cpu_req   = 1'b0;
            cpu_we    = 1'b0;
            cpu_size  = 2'b10;
            cpu_addr  = 32'hDEAD_BEEF;
            cpu_wstrb = 4'b0000;

            // 在 negedge 采样, 确保组合逻辑已稳定
            @(negedge clk);
            data = cpu_rdata;

            // 等一拍回到 S_IDLE
            @(posedge clk);

            $display("[TB] READ  addr=0x%08h → data=0x%08h  (stalled %0d cycles)",
                     addr, data, wait_cnt);
        end
    endtask

    // ── 写操作 ──
    task do_write;
        input  [31:0] addr;
        input  [31:0] wdata;
        input  [3:0]  wstrb;
        input  [1:0]  size;
        integer wait_cnt;
        begin
            cpu_req   = 1'b1;
            cpu_we    = 1'b1;
            cpu_addr  = addr;
            cpu_size  = size;
            cpu_wdata = wdata;
            cpu_wstrb = wstrb;

            @(posedge clk);
            wait_cnt = 1;

            while (cpu_stall !== 1'b0) begin
                @(posedge clk);
                wait_cnt = wait_cnt + 1;
            end

            // 先撤 cpu_req, 防止 DUT 误判为背靠背请求
            cpu_req   = 1'b0;
            cpu_we    = 1'b0;
            cpu_size  = 2'b10;
            cpu_addr  = 32'hDEAD_BEEF;
            cpu_wdata = 32'bx;
            cpu_wstrb = 4'b0000;

            @(negedge clk);
            @(posedge clk);

            $display("[TB] WRITE addr=0x%08h wdata=0x%08h wstrb=%b  (stalled %0d cycles)",
                     addr, wdata, wstrb, wait_cnt);
        end
    endtask

    // ============================================================
    // 仿真主流程
    // ============================================================
    reg  [31:0] rdata;
    reg  [31:0] b2b_addr;
    integer     cycle;

    initial begin
        // 初始化
        cpu_req   = 1'b0;
        cpu_we    = 1'b0;
        cpu_size  = 2'b10;
        cpu_addr  = 32'hDEAD_BEEF;
        cpu_wdata = 32'b0;
        cpu_wstrb = 4'b0000;

        // 等复位释放
        repeat (2) @(posedge clk);

        init_ext_mem();

        // 预热 1 个空闲拍 (确保状态机在 IDLE)
        @(posedge clk);

        $display("============================================================");
        $display("[TB] PHASE 1 — 读测试: 冷启动 miss → fill → hit");
        $display("============================================================");

        // ── T1: 读 addr(tag=0, word=0) — miss, fill way0 ──
        do_read(make_addr(19'd0, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd0, 4'd0))
            $display("  ** FAIL: expected 0x%08h, got 0x%08h", expected_word(19'd0, 4'd0), rdata);
        else
            $display("  ** PASS: read hit verified");

        // ── T2: 读 addr(tag=1, word=0) — miss, fill way1 ──
        do_read(make_addr(19'd1, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd1, 4'd0))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd1, 4'd0));
        else
            $display("  ** PASS");

        // ── T3: 读 addr(tag=0, word=0) — hit way0 ──
        do_read(make_addr(19'd0, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd0, 4'd0))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd0, 4'd0));
        else
            $display("  ** PASS: read hit way0");

        // ── T4: 读 addr(tag=1, word=0) — hit way1 ──
        do_read(make_addr(19'd1, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd1, 4'd0))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd1, 4'd0));
        else
            $display("  ** PASS: read hit way1");

        // ── T5: 读 addr(tag=0, word=5) — hit way0, 非零 word offset ──
        do_read(make_addr(19'd0, 4'd5, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd0, 4'd5))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd0, 4'd5));
        else
            $display("  ** PASS: read hit way0, word_offset=5");

        // ── T6: 读 addr(tag=1, word=15) — hit way1, 边界 word offset ──
        do_read(make_addr(19'd1, 4'd15, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd1, 4'd15))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd1, 4'd15));
        else
            $display("  ** PASS: read hit way1, word_offset=15");

        $display("============================================================");
        $display("[TB] PHASE 2 — 读测试: conflict miss, LRU eviction");
        $display("============================================================");

        // 当前状态: way0=tag0, way1=tag1
        // T3 hit way0 → lru=1 (way1 LRU)
        // T5 hit way0 → lru=1 (way1 still LRU)
        // T4 hit way1 → lru=0 (way0 LRU)
        // T6 hit way1 → lru=0 (way0 LRU)
        // → way0 is LRU

        // ── T7: 读 addr(tag=2, word=0) — miss, evict way0 (LRU) ──
        do_read(make_addr(19'd2, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd2, 4'd0))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd2, 4'd0));
        else
            $display("  ** PASS: conflict miss, LRU evict way0 → fill way0 with tag=2");

        // 当前: way0=tag2, way1=tag1. T7 hit way0 → lru=1 (way1 LRU)

        // ── T8: 读 addr(tag=1, word=0) — hit way1 ──
        do_read(make_addr(19'd1, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd1, 4'd0))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd1, 4'd0));
        else
            $display("  ** PASS: hit way1 still valid after eviction");

        // ── T9: 读 addr(tag=0, word=0) — miss, evict way1 (now LRU) ──
        //       tag=0 在 T7 被逐出了, 需要重新 fill
        do_read(make_addr(19'd0, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd0, 4'd0))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd0, 4'd0));
        else
            $display("  ** PASS: re-fetch evicted tag=0, LRU evict way1");

        // 当前: way0=tag2, way1=tag0

        // ── T10: 读 addr(tag=2, word=10) — hit way0 ──
        do_read(make_addr(19'd2, 4'd10, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd2, 4'd10))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd2, 4'd10));
        else
            $display("  ** PASS: hit way0, word offset 10");

        $display("============================================================");
        $display("[TB] PHASE 3 — 连读 4 tags 触发 2 次替换 (LRU shuffle)");
        $display("============================================================");

        // 当前: way0=tag2 (MRU), way1=tag0 (LRU)
        // T10 hit way0 → lru=1 (way1=tag0 is LRU)
        // → eviction would hit way1

        // ── T11: tag=3, miss, evict way1 (tag=0, LRU) ──
        do_read(make_addr(19'd3, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd3, 4'd0))
            $display("  ** FAIL");
        else
            $display("  ** PASS: evict tag=0 (LRU), fill tag=3 in way1");

        // 当前: way0=tag2, way1=tag3. T11 hit way1 → lru=0 (way0=tag2 LRU)

        // ── T12: tag=4, miss, evict way0 (tag=2, LRU) ──
        do_read(make_addr(19'd4, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd4, 4'd0))
            $display("  ** FAIL");
        else
            $display("  ** PASS: evict tag=2 (LRU), fill tag=4 in way0");

        // 当前: way0=tag4, way1=tag3. T12 hit way0 → lru=1 (way1=tag3 LRU)

        // ── T13: 读 tag=4 → hit way0 ──
        do_read(make_addr(19'd4, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd4, 4'd0))
            $display("  ** FAIL");
        else
            $display("  ** PASS: tag=4 still hit after shuffle");

        // ── T14: 读 tag=3 → hit way1 ──
        do_read(make_addr(19'd3, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd3, 4'd0))
            $display("  ** FAIL");
        else
            $display("  ** PASS: tag=3 still hit after shuffle");

        $display("============================================================");
        $display("[TB] PHASE 4 — 子字宽读测试 (byte / half)");
        $display("============================================================");

        // addr(tag=4, word=0, byte=0) — 读 byte 0
        // expected: word0 of tag4 = {8'd4, 8'd0, 8'd0, 8'd4}
        //   byte0 = 8'd4, byte1 = 8'd0, byte2 = 8'd0, byte3 = 8'd4
        // cache 返回整个 word, MEM stage 会做 sub-word 提取
        // 这里只测 cache 返回正确 word

        // ── T15: byte read at offset 0 ──
        do_read(make_addr(19'd4, 4'd0, 2'd0), 2'b00, rdata);
        if (rdata[7:0] !== 8'd4)
            $display("  ** FAIL: byte0 expected 0x%02h, got 0x%02h", 8'd4, rdata[7:0]);
        else
            $display("  ** PASS: byte read, rdata[7:0]=0x%02h", rdata[7:0]);

        // ── T16: byte read at offset 1 ──
        do_read(make_addr(19'd4, 4'd0, 2'd1), 2'b00, rdata);
        // byte1 of word0 = 8'd0
        if (rdata[7:0] !== 8'd0)
            $display("  ** FAIL: byte1 expected 0x00, got 0x%02h", rdata[7:0]);
        else
            $display("  ** PASS: byte read, rdata[7:0]=0x%02h", rdata[7:0]);

        // ── T17: halfword read at offset 2 ──
        do_read(make_addr(19'd4, 4'd0, 2'd2), 2'b01, rdata);
        // halfword at byte2 = {byte3, byte2} = {8'd4, 8'd0}
        if (rdata[15:0] !== {8'd4, 8'd0})
            $display("  ** FAIL: halfword expected 0x%04h", {8'd4, 8'd0});
        else
            $display("  ** PASS: halfword read");

        $display("============================================================");
        $display("[TB] PHASE 5 — 写测试: write hit + miss + write-allocate");
        $display("============================================================");

        // 当前: way0=tag4, way1=tag3.
        // T13 hit way0 → lru=1, T14 hit way1 → lru=0, T15-T17 hit way0
        // → way1=tag3 is LRU

        // ── T18: write hit way0 (tag=4, word=0) 全字写 ──
        do_write(make_addr(19'd4, 4'd0, 2'd0), 32'hDEAD_BEEF, 4'b1111, 2'b10);
        $display("  ** Write hit: way0 word0 → 0xDEAD_BEEF, dirty=1");

        // ── T19: read back to verify write ──
        do_read(make_addr(19'd4, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== 32'hDEAD_BEEF)
            $display("  ** FAIL: expected 0xDEAD_BEEF, got 0x%08h", rdata);
        else
            $display("  ** PASS: write hit verified by readback");

        // ── T20: write hit partial byte (wstrb=4'b0010, byte1 only) ──
        do_write(make_addr(19'd4, 4'd0, 2'd0), 32'h0000_AB00, 4'b0010, 2'b10);
        // expected: DEAD_BEEF with byte1=AB → DEAD_ABEF
        do_read(make_addr(19'd4, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== 32'hDEAD_ABEF)
            $display("  ** FAIL: partial write expected 0xDEAD_ABEF, got 0x%08h", rdata);
        else
            $display("  ** PASS: partial byte write (wstrb=0010)");

        // ── T21: write miss (tag=5, word=0) → write-allocate ──
        //       way1=tag3 is LRU. tag3 is clean.
        do_write(make_addr(19'd5, 4'd0, 2'd0), 32'hCAFE_BABE, 4'b1111, 2'b10);
        $display("  ** Write miss: write-allocate tag=5 in way1 (evict tag=3)");

        // ── T22: read back tag=5 → hit way1 ──
        do_read(make_addr(19'd5, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== 32'hCAFE_BABE)
            $display("  ** FAIL: expected 0xCAFE_BABE, got 0x%08h", rdata);
        else
            $display("  ** PASS: write-allocate verified");

        // ── T23: read tag=4 → still hit way0 ──
        do_read(make_addr(19'd4, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== 32'hDEAD_ABEF)
            $display("  ** FAIL: tag=4 changed after tag=5 write-allocate");
        else
            $display("  ** PASS: tag=4 still intact");

        // ── T24: write miss (tag=1, word=0) → write-allocate ──
        //       way0=tag4 (dirty!), way1=tag5 (dirty!)
        //       LRU: T23 hit way0 → lru=1 (way1=tag5 LRU)
        //       evict way1 (tag=5 dirty) → write back to ext_mem
        do_write(make_addr(19'd1, 4'd0, 2'd0), 32'h1111_2222, 4'b1111, 2'b10);
        $display("  ** Write miss: dirty evict tag=5 to ext_mem, fill tag=1");

        // ── T25: read tag=1 → hit ──
        do_read(make_addr(19'd1, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== 32'h1111_2222)
            $display("  ** FAIL: expected 0x1111_2222, got 0x%08h", rdata);
        else
            $display("  ** PASS: read after write-allocate");

        // ── T26: verify dirty write-back to ext_mem ──
        //       tag=5 should have been written back to ext_mem at word0 = 0xCAFE_BABE
        //       Wait... write-allocate for tag=1 should have evicted tag=5 (dirty).
        //       The entire tag=5 line should have been written back.
        //       Let's check tag=0 in ext_mem to confirm it's intact.
        //       And we can't read tag=5 from cache now (it was evicted).
        //       Read tag=5 again → miss → fill from ext_mem → should get back
        //       the dirty write-back value 0xCAFE_BABE for word0, and original
        //       initialized values for other words.
        do_read(make_addr(19'd5, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== 32'hCAFE_BABE)
            $display("  ** FAIL: dirty WB: word0 expected 0xCAFE_BABE, got 0x%08h", rdata);
        else
            $display("  ** PASS: dirty write-back verified (tag=5 word0=0xCAFE_BABE in ext_mem)");

        // word 5 of tag=5 should still be initialized value: {8'd5, 8'd5, 8'd5, 8'd5}
        do_read(make_addr(19'd5, 4'd5, 2'd0), 2'b10, rdata);
        if (rdata !== expected_word(19'd5, 4'd5))
            $display("  ** FAIL: dirty WB: word5 expected 0x%08h, got 0x%08h",
                     expected_word(19'd5, 4'd5), rdata);
        else
            $display("  ** PASS: dirty WB non-written words intact");

        $display("============================================================");
        $display("[TB] PHASE 6 — 背靠背 read→read (S_DATA→S_DATA, 0 bubble)");
        $display("============================================================");

        // 当前: way0=tag5, way1=tag1. 两个都 hit.
        // 交替读 tag5/word0 和 tag1/word0, 手动模拟流水线背靠背

        // B2B 时序: posedge 发起/锁存 → negedge 采样 combo 结果
        //   S_DATA 拍 stall=0, 下一拍自动推进到 S_DATA (背靠背)

        // ── req0: tag=5 word=0 ──
        cpu_req   = 1'b1;
        cpu_we    = 1'b0;
        cpu_addr  = make_addr(19'd5, 4'd0, 2'd0);
        cpu_size  = 2'b10;
        cpu_wstrb = 4'b0000;
        @(posedge clk);                                    // S_IDLE→S_DATA (req0), BRAM 数据锁存
        b2b_addr  = cpu_addr;
        cpu_addr  = make_addr(19'd1, 4'd0, 2'd0);         // 切 addr (模拟 EX/MEM 更新)
        @(negedge clk);                                    // combo 稳定, 采样 req0 数据
        rdata = cpu_rdata;
        if (rdata !== 32'hCAFE_BABE)
            $display("  ** FAIL: B2B-0 expected 0xCAFE_BABE, got 0x%08h", rdata);
        else
            $display("  ** PASS: B2B-0 tag=5/word0 → 0x%08h", rdata);

        // ── req1: tag=1 word=0 (已在上一 S_DATA 被接受) ──
        @(posedge clk);                                    // S_DATA→S_DATA (req1), BRAM 锁存
        b2b_addr  = cpu_addr;
        cpu_addr  = make_addr(19'd5, 4'd5, 2'd0);
        @(negedge clk);
        rdata = cpu_rdata;
        if (rdata !== 32'h11112222)
            $display("  ** FAIL: B2B-1 expected 0x11112222, got 0x%08h", rdata);
        else
            $display("  ** PASS: B2B-1 tag=1/word0 → 0x%08h", rdata);

        // ── req2: tag=5 word=5 ──
        @(posedge clk);
        b2b_addr  = cpu_addr;
        cpu_addr  = make_addr(19'd1, 4'd10, 2'd0);
        @(negedge clk);
        rdata = cpu_rdata;
        if (rdata !== 32'h05050505)
            $display("  ** FAIL: B2B-2 expected 0x05050505, got 0x%08h", rdata);
        else
            $display("  ** PASS: B2B-2 tag=5/word5 → 0x%08h", rdata);

        // 收尾
        @(posedge clk);                                    // 最后一级 S_DATA→S_IDLE
        cpu_req   = 1'b0;
        @(posedge clk);

        $display("============================================================");
        $display("[TB] PHASE 7 — 背靠背 read→write hit (S_DATA→S_HIT_WR)  ");
        $display("============================================================");

        // 当前: way0=tag5, way1=tag1. tag5 dirty=0.
        // 手动时序: posedge 发请求 → posedge 后切信号 → 等 S_HIT_WR

        cpu_req   = 1'b1;
        cpu_we    = 1'b0;
        cpu_addr  = make_addr(19'd5, 4'd0, 2'd0);
        cpu_size  = 2'b10;
        cpu_wstrb = 4'b0000;
        @(posedge clk);                                    // S_IDLE→S_DATA (read)
        // S_DATA 拍: 切信号为写, S_DATA 接受新请求 → next=S_HIT_WR
        cpu_we    = 1'b1;
        cpu_wdata = 32'hBEEF_DEAD;
        cpu_wstrb = 4'b1111;
        cpu_addr  = make_addr(19'd5, 4'd0, 2'd0);         // 同地址写
        @(posedge clk);                                    // S_DATA→S_HIT_WR
        // S_HIT_WR: RMW 完成, stall=0
        cpu_req   = 1'b0;
        cpu_we    = 1'b0;
        @(posedge clk);                                    // S_HIT_WR→S_IDLE

        // 验证: 读回 tag=5/word0
        do_read(make_addr(19'd5, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== 32'hBEEF_DEAD)
            $display("  ** FAIL: read→write hit: expected 0xBEEF_DEAD, got 0x%08h", rdata);
        else
            $display("  ** PASS: read→write hit (S_DATA→S_HIT_WR), dirty verified");

        $display("============================================================");
        $display("[TB] PHASE 8 — 背靠背 read→write miss (S_DATA→EVICT/FILL)");
        $display("============================================================");

        // 当前: way0=tag5(dirty), way1=tag1(dirty). LRU: way1=tag1 is LRU.
        // 发 read tag=5/word0, S_DATA 拍切 write tag=6/word0 (miss)
        // → sdata_miss=1, stall=1 → EVICT way1(tag1) → FILL tag=6 → S_HIT_WR

        cpu_req   = 1'b1;
        cpu_we    = 1'b0;
        cpu_addr  = make_addr(19'd5, 4'd0, 2'd0);
        cpu_size  = 2'b10;
        cpu_wstrb = 4'b0000;
        @(posedge clk);                                    // S_IDLE→S_DATA (read tag=5)
        // S_DATA 拍: 切为 write miss
        cpu_we    = 1'b1;
        cpu_wdata = 32'hFEED_FACE;
        cpu_wstrb = 4'b1111;
        cpu_addr  = make_addr(19'd6, 4'd0, 2'd0);         // tag=6, miss! sdata_miss=1
        while (cpu_stall !== 1'b0) @(posedge clk);         // 等 EVICT→FILL→RETRY→S_HIT_WR
        // S_HIT_WR: stall=0, write-allocate 完成
        cpu_req   = 1'b0;
        cpu_we    = 1'b0;
        @(posedge clk);                                    // S_HIT_WR→S_IDLE

        // 验证: 读 tag=6/word0, 应得到 0xFEED_FACE
        do_read(make_addr(19'd6, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== 32'hFEED_FACE)
            $display("  ** FAIL: read→write miss: expected 0xFEED_FACE, got 0x%08h", rdata);
        else
            $display("  ** PASS: read→write miss (S_DATA→EVICT→FILL→S_HIT_WR)");

        // 验证: tag=5 未受损
        do_read(make_addr(19'd5, 4'd0, 2'd0), 2'b10, rdata);
        if (rdata !== 32'hBEEF_DEAD)
            $display("  ** FAIL: tag=5 corrupted after read→write miss, got 0x%08h", rdata);
        else
            $display("  ** PASS: tag=5 intact after read→write miss");

        $display("============================================================");
        $display("[TB] All tests complete.");
        $display("============================================================");

        #(CLK_PERIOD * 5);
        $finish;
    end

    // ============================================================
    // 周期计数
    // ============================================================
    initial begin
        cycle = 0;
        forever begin
            @(posedge clk);
            cycle = cycle + 1;
        end
    end

endmodule
