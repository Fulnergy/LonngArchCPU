//数据存储(或缓存)直接例化在此模块下
module MEM_Stage(
    input clk,
    input en,
    input [31:0] data_addr,
    input [31:0] write_data, //store将要写入的数据
    output [31:0] read_data //load读出的数据
);

endmodule