//由于WB也需要访问寄存器，寄存器堆例化在RegControll中。
//该模块解码后，将寄存器地址输出到top，寄存器读取的结果直接从top向后面的stage传递。
module ID_Stage(
    input wire clk,
    input wire en,
    input wire [63:0] dual_inst,
    //两条指令，分别记作0和1,其中仅1可进行LS
    output wire [31:0] sig0_id, sig1_id,
    output wire [4:0] reg01,reg02,reg11,reg12,
    output wire [31:0] imm1,imm2
);


endmodule