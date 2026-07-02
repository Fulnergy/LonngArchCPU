module RegControll(
    input clk,
    input en,
    input regWrite0, regWrite1,
    input [4:0] write_addr0, write_addr1, //写入地址
    input [4:0] read_addr01, read_addr02, read_addr11, read_addr12,//读取地址
    input [31:0] write_data0, write_data1, //写入数据
    output [31:0] read01, read02, read11, read12 //读取数据

);

endmodule