module Decoder (
    input wire [31:0] inst,
    output wire [4:0] aluOp,
    output wire [31:0] imm,
    output wire [4:0] rd,
    output wire [4:0] rj,
    output wire [4:0] rk,
    output wire memRead, memWrite,
    output wire branch,
    output wire jump,//branch以外的跳转
    output wire regWrite
);


endmodule