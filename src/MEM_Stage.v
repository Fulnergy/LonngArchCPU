// 数据存储器 —— 例化 dMem 模块
module MEM_Stage #(
    parameter ADDR_WIDTH = 15,               // 地址位宽 (字地址)
    parameter DATA_WIDTH = 32,               // 数据位宽
    parameter INIT_FILE  = "dmem_init.hex"   // 仿真初始化文件
)(
    input  wire                      clk,
    input  wire                      wr_en,        // 写使能: 1=写入, 0=读取
    input  wire [3:0]                byte_we,      // 字节写使能: bit[i] 使能第 i 个字节的写入
    input  wire [ADDR_WIDTH-1:0]     data_addr,
    input  wire [DATA_WIDTH-1:0]     write_data,   // store 写入数据
    output wire [DATA_WIDTH-1:0]     read_data     // load  读出数据
);

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
        .dout    (read_data)
    );

endmodule