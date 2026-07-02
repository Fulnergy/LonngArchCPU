module EX_ALU(
    input clk,
    input en,
    input [4:0] aluOp,
    input [31:0] reg1,reg2,//寄存器1和2的值
    input [31:0] imm,
    //alu则为结果，branch则为0(不跳)或1(跳)
    output [31:0] result
)

endmodule