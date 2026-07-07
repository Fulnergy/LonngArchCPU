// 数据存储器 —— 例化 dMem, byte_we 兼作读写掩码, signExt 控制符号/零扩展
module MEM_Stage #(
    parameter ADDR_WIDTH = 15,               // 地址位宽 (字地址)
    parameter DATA_WIDTH = 32,               // 数据位宽
    parameter INIT_FILE  = "dmem_init.hex"   // 仿真初始化文件
)(
    input  wire                      clk,
    input  wire                      wr_en,        // 写使能: 1=写入, 0=读取
    input  wire                      signExt,      // 0=符号扩展, 1=零扩展
    input  wire [3:0]                byte_we,      // 字节使能: store→写掩码, load→读掩码
    input  wire [ADDR_WIDTH-1:0]     data_addr,
    input  wire [DATA_WIDTH-1:0]     write_data,   // store 写入数据
    output wire [DATA_WIDTH-1:0]     read_data     // load 读出数据 (已扩展)
);

    // dmem 原始输出 (全 32bit)
    wire [DATA_WIDTH-1:0] dmem_dout_raw;

    // 例化数据存储器 BRAM
    dMem #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH),
        .INIT_FILE  (INIT_FILE)
    ) u_dmem (
        .clk     (clk),
        .writeEn (wr_en),
        .byteWe  (byte_we),
        .addr    (data_addr),
        .din     (write_data),
        .dout    (dmem_dout_raw)
    );

    // ── 读数据字节掩码 ──
    // byte_we[i]=1 时对应字节直通, =0 时清零
    // LB: 仅 1 字节有效; LH: 2 字节; LW: 全 4 字节
    wire [DATA_WIDTH-1:0] byte_mask;
    assign byte_mask = {{8{byte_we[3]}}, {8{byte_we[2]}}, {8{byte_we[1]}}, {8{byte_we[0]}}};

    // 掩码后的数据
    wire [DATA_WIDTH-1:0] masked_data;
    assign masked_data = dmem_dout_raw & byte_mask;

    // ── 符号位定位 ──
    // 取 byte_we 中最高有效字节 → 其 MSB 即为符号位
    wire [4:0] sign_bit;
    assign sign_bit = byte_we[3] ? 5'd31 :
                      byte_we[2] ? 5'd23 :
                      byte_we[1] ? 5'd15 :
                                   5'd7;   // byte_we[0] 必为 1

    // ── 符号/零扩展 ──
    // signExt=0: 符号扩展 → 高位填充 sign_bit
    // signExt=1: 零扩展  → 高位填 0 (byte_mask 已清零)
    wire [31:0] sign_fill;
    assign sign_fill = {32{masked_data[sign_bit]}} & ~byte_mask;
    assign read_data = signExt ? masked_data : (masked_data | sign_fill);

endmodule