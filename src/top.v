//双发射，槽0可branch,槽1可load/store，二者都兼容alu
module top(
    input clk
);

IF_Stage uif(

);

// ID Stage 输出
wire [9:0]  opc0_id, opc1_id;
wire [6:0]  func0_id, func1_id;
wire [4:0]  sigs0_id, sigs1_id;
wire [31:0] imm0, imm1;
wire [14:0] regs0_id, regs1_id;

ID_stage uid(
    .dual_inst,
    .opc0(opc0_id),
    .opc1(opc1_id),
    .func0(func0_id),
    .func1(func1_id),
    .sigs0(sigs0_id),
    .sigs1(sigs1_id),
    .imm0(imm0),
    .imm1(imm1),
    .regs0(regs0_id),
    .regs1(regs1_id)
);


// ── Regs 写端口 (待 WB 级驱动) ──
wire [4:0]  wb_waddr0, wb_waddr1;       // 写回地址 (rd)
wire [31:0] wb_wdata0, wb_wdata1;       // 写回数据
wire        wb_rw0, wb_rw1;             // 写使能

// ── Regs 读端口数据 ──
wire [31:0] rdata01, rdata02;           // 槽0 读出 (rj, rk)
wire [31:0] rdata11, rdata12;           // 槽1 读出 (rj, rk)

Regs urg(
    .clk(clk),
    .en(1'b1),
    // 写端口 (WB → Regs)
    .regWrite0(wb_rw0),
    .regWrite1(wb_rw1),
    .write_addr0(wb_waddr0),
    .write_addr1(wb_waddr1),
    .write_data0(wb_wdata0),
    .write_data1(wb_wdata1),
    // 读端口 (ID → Regs, 地址直连)
    .read_addr01(regs0_id[9:5]),        // 槽0 rs = rj
    .read_addr02(regs0_id[14:10]),      // 槽0 rt = rk
    .read_addr11(regs1_id[9:5]),        // 槽1 rs = rj
    .read_addr12(regs1_id[14:10]),      // 槽1 rt = rk
    // 读出数据
    .read01(rdata01),
    .read02(rdata02),
    .read11(rdata11),
    .read12(rdata12)
);

// ── EX 级信号 ──
wire [31:0] alu_result0, alu_result1;
wire        jump_taken0;
wire [31:0] jump_addr0;
wire [31:0] pc;                       // TODO: 由 IF_Stage 驱动

// ============================================================
// ID -> EX 流水线寄存器
// ============================================================
reg [9:0]  opc0_ex, opc1_ex;
reg [6:0]  func0_ex, func1_ex;
reg [4:0]  sigs0_ex, sigs1_ex;
reg [14:0] regs0_ex, regs1_ex;
reg [31:0] imm0_ex, imm1_ex;
reg [31:0] pc_ex;
reg [31:0] rd01_ex, rd02_ex;          // 槽0 寄存器值 (流水线后)
reg [31:0] rd11_ex, rd12_ex;          // 槽1 寄存器值 (流水线后)

always @(posedge clk) begin
    // 控制 & 译码
    sigs0_ex <= sigs0_id;
    sigs1_ex <= sigs1_id;
    opc0_ex  <= opc0_id;
    opc1_ex  <= opc1_id;
    func0_ex <= func0_id;
    func1_ex <= func1_id;
    imm0_ex  <= imm0;
    imm1_ex  <= imm1;
    // 寄存器地址
    regs0_ex <= regs0_id;
    regs1_ex <= regs1_id;
    // 寄存器值
    rd01_ex <= rdata01;
    rd02_ex <= rdata02;
    rd11_ex <= rdata11;
    rd12_ex <= rdata12;
    // PC
    pc_ex   <= pc;
end

// 对应槽0: ALU / Branch (使用流水线后 EX 级信号)
EX_ALU uea(
    .clk(clk),
    .en(1'b1),
    .branch(sigs0_ex[3]),
    .jump(sigs0_ex[4]),
    .opcode(opc0_ex),
    .func(func0_ex),
    .reg1(rd01_ex),
    .reg2(rd02_ex),
    .imm(imm0_ex),
    .pc(pc_ex),
    .alu_result(alu_result0),
    .jump_taken(jump_taken0),
    .jump_addr(jump_addr0)
);



endmodule