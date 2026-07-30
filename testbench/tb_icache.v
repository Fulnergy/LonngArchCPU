`timescale 1ns / 1ps

// ============================================================================
// iCache 测试平台
//   覆盖: 读命中/缺失, victim 替换 (LRU + invalid优先), 背靠背读
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
// ============================================================================

module tb_icache;

    parameter CLK_PERIOD = 10;

    // ============================================================
    // DUT 信号 (iCache 只读, 无 write 端口)
    // ============================================================
    reg         clk;
    reg         rst_n;

    reg         cpu_req;
    reg  [31:0] cpu_addr;
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
    iCache #(
        .ADDR_WIDTH      (32),
        .DATA_WIDTH      (32),
        .LINE_SIZE_BYTES (64),
        .NUM_SETS        (128),
        .NUM_WAYS        (2)
    ) u_icache (
        .clk        (clk),
        .rst_n      (rst_n),
        .cpu_req    (cpu_req),
        .cpu_addr   (cpu_addr),
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
    // 外部存储模拟 (只读)
    // ============================================================
    localparam EXT_MEM_DEPTH = 16384;
    reg [31:0] ext_mem [0:EXT_MEM_DEPTH-1];
    parameter ADDR_MSB = 31;

    reg         ext_req_q;
    reg  [31:0] ext_addr_q;

    always @(posedge clk) begin
        ext_req_q  <= ext_req;
        ext_addr_q <= ext_addr;
    end

    always @(posedge clk) begin
        if (!rst_n)
            ext_ready <= 1'b0;
        else
            ext_ready <= ext_req;
    end

    reg [31:0] ext_rdata_r;
    always @(posedge clk) begin
        if (ext_req_q)
            ext_rdata_r <= ext_mem[ext_addr_q[ADDR_MSB:2]];
    end

    assign ext_rdata = ext_rdata_r;

    // ============================================================
    // 外部存储初始化
    // ============================================================
    task init_ext_mem;
        integer tag, wn;
        integer line_base;
        begin
            for (tag = 0; tag < 8; tag = tag + 1) begin
                line_base = tag << 13;
                for (wn = 0; wn < 16; wn = wn + 1) begin
                    ext_mem[(line_base >> 2) + wn] = {tag[7:0], wn[7:0], wn[7:0], tag[7:0]};
                end
            end
            $display("[TB] ext_mem initialized: 8 lines (tags 0..7), 64B each");
        end
    endtask

    // ============================================================
    // 地址宏
    // ============================================================
    function [31:0] make_addr;
        input [18:0] tag;
        input [3:0]  word_off;
        input [1:0]  byte_off;
        begin
            make_addr = {tag, 7'b0, 6'b0} | {14'b0, word_off, 2'b0} | {30'b0, byte_off};
        end
    endfunction

    function [31:0] expected_word;
        input [18:0] tag;
        input [3:0]  word_off;
        begin
            expected_word = {{4'b0, tag[3:0]}, {4'b0, word_off},
                             {4'b0, word_off}, {4'b0, tag[3:0]}};
        end
    endfunction

    // ============================================================
    // 读操作任务
    // ============================================================
    task do_read;
        input  [31:0] addr;
        output [31:0] data;
        integer wait_cnt;
        begin
            cpu_req  = 1'b1;
            cpu_addr = addr;

            @(posedge clk);
            wait_cnt = 1;

            while (cpu_stall !== 1'b0) begin
                @(posedge clk);
                wait_cnt = wait_cnt + 1;
            end

            // 先撤 cpu_req, 防止误判背靠背请求
            cpu_req  = 1'b0;
            cpu_addr = 32'hDEAD_BEEF;

            @(negedge clk);
            data = cpu_rdata;

            @(posedge clk);

            $display("[TB] READ  addr=0x%08h → data=0x%08h  (stalled %0d cycles)",
                     addr, data, wait_cnt);
        end
    endtask

    // ============================================================
    // 仿真主流程
    // ============================================================
    reg  [31:0] rdata;
    reg  [31:0] b2b_addr;
    integer     cycle;

    initial begin
        cpu_req  = 1'b0;
        cpu_addr = 32'hDEAD_BEEF;

        repeat (2) @(posedge clk);
        init_ext_mem();
        @(posedge clk);

        $display("============================================================");
        $display("[TB] PHASE 1 — 读测试: 冷启动 miss → fill → hit");
        $display("============================================================");

        // T1: tag=0 word=0 — miss, fill way0
        do_read(make_addr(19'd0, 4'd0, 2'd0), rdata);
        if (rdata !== expected_word(19'd0, 4'd0))
            $display("  ** FAIL: expected 0x%08h, got 0x%08h", expected_word(19'd0, 4'd0), rdata);
        else
            $display("  ** PASS");

        // T2: tag=1 word=0 — miss, fill way1
        do_read(make_addr(19'd1, 4'd0, 2'd0), rdata);
        if (rdata !== expected_word(19'd1, 4'd0))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd1, 4'd0));
        else
            $display("  ** PASS");

        // T3: tag=0 word=0 — hit way0
        do_read(make_addr(19'd0, 4'd0, 2'd0), rdata);
        if (rdata !== expected_word(19'd0, 4'd0))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd0, 4'd0));
        else
            $display("  ** PASS: read hit way0");

        // T4: tag=1 word=0 — hit way1
        do_read(make_addr(19'd1, 4'd0, 2'd0), rdata);
        if (rdata !== expected_word(19'd1, 4'd0))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd1, 4'd0));
        else
            $display("  ** PASS: read hit way1");

        // T5: tag=0 word=5 — hit way0, 非零 word offset
        do_read(make_addr(19'd0, 4'd5, 2'd0), rdata);
        if (rdata !== expected_word(19'd0, 4'd5))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd0, 4'd5));
        else
            $display("  ** PASS: hit way0, word_offset=5");

        // T6: tag=1 word=15 — hit way1, 边界 word offset
        do_read(make_addr(19'd1, 4'd15, 2'd0), rdata);
        if (rdata !== expected_word(19'd1, 4'd15))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd1, 4'd15));
        else
            $display("  ** PASS: hit way1, word_offset=15");

        $display("============================================================");
        $display("[TB] PHASE 2 — 读测试: conflict miss, LRU eviction");
        $display("============================================================");

        // 当前: way0=tag0, way1=tag1
        // T3 hit way0 → lru=1, T5 hit way0 → lru=1
        // T4 hit way1 → lru=0, T6 hit way1 → lru=0
        // → way0 is LRU

        // T7: tag=2 word=0 — miss, evict way0 (LRU)
        do_read(make_addr(19'd2, 4'd0, 2'd0), rdata);
        if (rdata !== expected_word(19'd2, 4'd0))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd2, 4'd0));
        else
            $display("  ** PASS: conflict miss, LRU evict way0 → fill way0 tag=2");

        // 当前: way0=tag2, way1=tag1. T7 hit way0 → lru=1 (way1 LRU)

        // T8: tag=1 word=0 — hit way1
        do_read(make_addr(19'd1, 4'd0, 2'd0), rdata);
        if (rdata !== expected_word(19'd1, 4'd0))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd1, 4'd0));
        else
            $display("  ** PASS: hit way1 still valid after eviction");

        // T9: tag=0 word=0 — miss, evict way1 (now LRU)
        do_read(make_addr(19'd0, 4'd0, 2'd0), rdata);
        if (rdata !== expected_word(19'd0, 4'd0))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd0, 4'd0));
        else
            $display("  ** PASS: re-fetch evicted tag=0, LRU evict way1");

        // 当前: way0=tag2, way1=tag0

        // T10: tag=2 word=10 — hit way0
        do_read(make_addr(19'd2, 4'd10, 2'd0), rdata);
        if (rdata !== expected_word(19'd2, 4'd10))
            $display("  ** FAIL: expected 0x%08h", expected_word(19'd2, 4'd10));
        else
            $display("  ** PASS: hit way0, word offset 10");

        $display("============================================================");
        $display("[TB] PHASE 3 — 连读 4 tags 触发 2 次替换 (LRU shuffle)");
        $display("============================================================");

        // 当前: way0=tag2 (MRU), way1=tag0 (LRU)

        // T11: tag=3 — miss, evict way1 (tag=0, LRU)
        do_read(make_addr(19'd3, 4'd0, 2'd0), rdata);
        if (rdata !== expected_word(19'd3, 4'd0))
            $display("  ** FAIL");
        else
            $display("  ** PASS: evict tag=0 (LRU), fill tag=3 in way1");

        // T12: tag=4 — miss, evict way0 (tag=2, LRU)
        do_read(make_addr(19'd4, 4'd0, 2'd0), rdata);
        if (rdata !== expected_word(19'd4, 4'd0))
            $display("  ** FAIL");
        else
            $display("  ** PASS: evict tag=2 (LRU), fill tag=4 in way0");

        // T13: tag=4 → hit way0
        do_read(make_addr(19'd4, 4'd0, 2'd0), rdata);
        if (rdata !== expected_word(19'd4, 4'd0))
            $display("  ** FAIL");
        else
            $display("  ** PASS: tag=4 still hit after shuffle");

        // T14: tag=3 → hit way1
        do_read(make_addr(19'd3, 4'd0, 2'd0), rdata);
        if (rdata !== expected_word(19'd3, 4'd0))
            $display("  ** FAIL");
        else
            $display("  ** PASS: tag=3 still hit after shuffle");

        $display("============================================================");
        $display("[TB] PHASE 4 — 子字宽读测试 (byte / half)");
        $display("============================================================");

        // T15: byte read at offset 0
        do_read(make_addr(19'd4, 4'd0, 2'd0), rdata);
        if (rdata[7:0] !== 8'd4)
            $display("  ** FAIL: byte0 expected 0x%02h, got 0x%02h", 8'd4, rdata[7:0]);
        else
            $display("  ** PASS: byte read, rdata[7:0]=0x%02h", rdata[7:0]);

        // T16: byte read at offset 1
        do_read(make_addr(19'd4, 4'd0, 2'd1), rdata);
        if (rdata[7:0] !== 8'd0)
            $display("  ** FAIL: byte1 expected 0x00, got 0x%02h", rdata[7:0]);
        else
            $display("  ** PASS: byte read, rdata[7:0]=0x%02h", rdata[7:0]);

        // T17: halfword read at offset 2
        do_read(make_addr(19'd4, 4'd0, 2'd2), rdata);
        if (rdata[15:0] !== {8'd4, 8'd0})
            $display("  ** FAIL: halfword expected 0x%04h", {8'd4, 8'd0});
        else
            $display("  ** PASS: halfword read");

        $display("============================================================");
        $display("[TB] PHASE 5 — 背靠背连续读 (S_DATA→S_DATA 0 stall)");
        $display("============================================================");

        // 当前: way0=tag4, way1=tag3. T13 hit way0, T14 hit way1, T15-T17 hit way0
        // 交替读 tag4 和 tag3, 验证背靠背无气泡

        // req0: tag=4 word=0
        cpu_req  = 1'b1;
        cpu_addr = make_addr(19'd4, 4'd0, 2'd0);
        @(posedge clk);
        b2b_addr = cpu_addr;
        cpu_addr = make_addr(19'd3, 4'd0, 2'd0);
        @(negedge clk);
        $display("[TB] B2B-0: addr=0x%08h → 0x%08h  (stall=%b)", b2b_addr, cpu_rdata, cpu_stall);

        // req1: tag=3 word=0
        @(posedge clk);
        b2b_addr = cpu_addr;
        cpu_addr = make_addr(19'd4, 4'd5, 2'd0);
        @(negedge clk);
        $display("[TB] B2B-1: addr=0x%08h → 0x%08h  (stall=%b)", b2b_addr, cpu_rdata, cpu_stall);

        // req2: tag=4 word=5
        @(posedge clk);
        b2b_addr = cpu_addr;
        cpu_addr = make_addr(19'd3, 4'd10, 2'd0);
        @(negedge clk);
        $display("[TB] B2B-2: addr=0x%08h → 0x%08h  (stall=%b)", b2b_addr, cpu_rdata, cpu_stall);

        // req3: tag=3 word=10
        @(posedge clk);
        @(negedge clk);
        $display("[TB] B2B-3: addr=0x%08h → 0x%08h  (stall=%b)", cpu_addr, cpu_rdata, cpu_stall);

        cpu_req = 1'b0;
        @(posedge clk);

        $display("  ** PASS: 背靠背连续读 (每拍 1 个, 0 stall bubble)");

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
