// 数据存储器 —— 例化 dMem, 内部译码 byte_we, signExt 控制符号/零扩展
module MEM_Stage #(
    parameter ADDR_WIDTH = 17,               // 字节地址位宽 (17b → 128KB)
    parameter DATA_WIDTH = 32,               // 数据位宽
    parameter INIT_FILE  = "dmem_init.hex"   // 仿真初始化文件
)(
    input  wire                      clk,
    input  wire                      wr_en,        // 写使能: 1=写入, 0=读取
    input  wire                      signExt,      // 0=符号扩展, 1=零扩展
    input  wire [1:0]                mem_size,     // 访存宽度: 00=byte, 01=half, 10=word
    input  wire [ADDR_WIDTH-1:0]     data_addr,    // 字节地址
    input  wire [DATA_WIDTH-1:0]     write_data,   // store 写入数据
    output wire [DATA_WIDTH-1:0]     read_data     // load 读出数据 (已扩展)
);

    localparam WORD_WIDTH = ADDR_WIDTH - 2;  // 字地址位宽 (17→15)

    // ── byte_we 译码: mem_size + 字节偏移 → 字节使能 ──
    wire [3:0] byte_we;
    assign byte_we =
        (mem_size == 2'b00) ? (4'b0001 << data_addr[1:0]) :        // byte
        (mem_size == 2'b01) ? (4'b0011 << {data_addr[1], 1'b0}) :  // halfword
                              4'b1111;                              // word

    // dmem 原始输出 (全 32bit)
    wire [DATA_WIDTH-1:0] dmem_dout_raw;

    // 字节地址 → 字地址
    wire [WORD_WIDTH-1:0] word_addr;
    assign word_addr = data_addr[ADDR_WIDTH-1:2];

    // ── 写数据对齐: reg 低位 → 内存字内对应字节位置 (左移) ──
    wire [DATA_WIDTH-1:0] aligned_wdata;
    assign aligned_wdata = write_data << shift_amt;

    // 例化数据存储器 BRAM (字地址接口)
    dMem #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (WORD_WIDTH),
        .INIT_FILE  (INIT_FILE)
    ) u_dmem (
        .clk     (clk),
        .writeEn (wr_en),
        .byteWe  (byte_we),
        .addr    (word_addr),
        .din     (aligned_wdata),
        .dout    (dmem_dout_raw)
    );

    // ── 读数据字节掩码 ──
    // byte_we[i]=1 时对应字节直通, =0 时清零
    wire [DATA_WIDTH-1:0] byte_mask;
    assign byte_mask = {{8{byte_we[3]}}, {8{byte_we[2]}}, {8{byte_we[1]}}, {8{byte_we[0]}}};

    // 掩码后的数据 (保持原字内位置)
    wire [DATA_WIDTH-1:0] masked_data;
    assign masked_data = dmem_dout_raw & byte_mask;

    // ── 低位对齐: 右移使数据对齐到 reg 低字节 ──
    // 找到 byte_we 中最低置位 → 对应字节偏移 → 右移 offset*8
    wire [4:0] shift_amt;
    assign shift_amt = byte_we[0] ? 5'd0  :
                       byte_we[1] ? 5'd8  :
                       byte_we[2] ? 5'd16 :
                                    5'd24;

    wire [DATA_WIDTH-1:0] aligned_data;
    assign aligned_data = masked_data >> shift_amt;

    // ── 数据宽度检测 (byte_we 置位数: 1→byte, 2→half, 4→word) ──
    wire [2:0] byte_cnt;
    assign byte_cnt = byte_we[0] + byte_we[1] + byte_we[2] + byte_we[3];

    wire [4:0] data_msb;               // 对齐后数据的最高有效位
    assign data_msb = (&byte_we)       ? 5'd31 :   // word
                      byte_cnt[1]      ? 5'd15 :   // halfword (cnt=2)
                                         5'd7;     // byte (cnt=1)

    // ── 符号/零扩展 ──
    // signExt=0: 符号扩展 → 高位填充 data_msb 的值
    // signExt=1: 零扩展  → 高位清零 (移位已自然清零)
    wire [31:0] upper_mask;            // data_msb 以上的位
    assign upper_mask = 32'hFFFFFFFF << (data_msb + 1);
    wire [31:0] sign_fill;
    assign sign_fill = {32{aligned_data[data_msb]}} & upper_mask;

    assign read_data = signExt ? aligned_data : (aligned_data | sign_fill);

endmodule