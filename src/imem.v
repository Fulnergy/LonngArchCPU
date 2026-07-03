module imem #(
    parameter ADDR_WIDTH = 13,                // 字节地址位宽 (13 bits → 8KB)
    parameter DATA_WIDTH = 64,                // 数据位宽 (64b 双发射)
    parameter INIT_FILE  = "imem_init.hex"    // 仿真初始化文件
)(
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire [ADDR_WIDTH-1:0]     addr,     // 字节地址
    output reg  [DATA_WIDTH-1:0]     dout
);

    localparam BYTE_SHIFT       = $clog2(DATA_WIDTH / 8);             // 32b→2, 64b→3
    localparam WORD_ADDR_WIDTH  = ADDR_WIDTH - BYTE_SHIFT;            // 字地址位宽
    localparam MEM_DEPTH        = 1 << WORD_ADDR_WIDTH;               // 存储深度

    // BRAM 存储阵列: 每项 DATA_WIDTH 位宽, 深度 = 字节空间 / 字宽
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] mem [0:MEM_DEPTH-1];

    // 仿真初始化
    initial begin
        $readmemh(INIT_FILE, mem);
    end

    // 同步 BRAM 读: addr 为字节地址, 内部按字对齐取指
    // NO reset on dout — BRAM 输出寄存器不支持异步复位
    always @(posedge clk) begin
        dout <= mem[addr[ADDR_WIDTH-1:BYTE_SHIFT]];
    end

endmodule