module ID_stage(
    input [63:0] dual_inst,
    output [9:0] opc0, opc1,        // opcode
    output [6:0] func0, func1,      // func  
    output [4:0] sigs0, sigs1,      // control bus: jump/branch/memRead/memWrite/regWrite
    output [31:0] imm0, imm1,
    output [19:0] regs0, regs1      // regs address: rk[14:10] | rj[9:5] | rd[4:0]
);

Decoder ud0(
    .inst(dual_inst[31:0]),
    .opcode(opc0),
    .func(func0),
    .imm(imm0),
    .rd(regs0[4:0]),
    .rj(regs0[9:5]),
    .rk(regs0[14:10]),
    .memRead(sigs0[2]),
    .memWrite(sigs0[1]),
    .branch(sigs0[3]),
    .jump(sigs0[4]),
    .regWrite(sigs0[0])
);

Decoder ud1(
    .inst(dual_inst[63:32]),
    .opcode(opc1),
    .func(func1),
    .imm(imm1),
    .rd(regs1[4:0]),
    .rj(regs1[9:5]),
    .rk(regs1[14:10]),
    .memRead(sigs1[2]),
    .memWrite(sigs1[1]),
    .branch(sigs1[3]),
    .jump(sigs1[4]),
    .regWrite(sigs1[0])
);

endmodule