module ID_stage(
    input [63:0] dual_inst,
    output reg [9:0] opc0, opc1,        // opcode
    output reg [6:0] func0, func1,      // func
    output reg [4:0] sigs0, sigs1,      // {jump, branch, memRead, memWrite, regWrite}
    output reg [31:0] imm0, imm1,
    output reg [14:0] regs0, regs1,     // {rk, rj, rd}
    output reg nop0, nop1
);

    // ============================================================
    // Decoder 原始输出 (raw_*)
    // ============================================================
    wire [9:0]  raw_opc0, raw_opc1;
    wire [6:0]  raw_func0, raw_func1;
    wire [4:0]  raw_sigs0, raw_sigs1;
    wire [31:0] raw_imm0, raw_imm1;
    wire [14:0] raw_regs0, raw_regs1;

    Decoder ud0(
        .inst(dual_inst[31:0]),
        .opcode(raw_opc0),
        .func(raw_func0),
        .imm(raw_imm0),
        .rd(raw_regs0[4:0]),
        .rj(raw_regs0[9:5]),
        .rk(raw_regs0[14:10]),
        .memRead(raw_sigs0[2]),
        .memWrite(raw_sigs0[1]),
        .branch(raw_sigs0[3]),
        .jump(raw_sigs0[4]),
        .regWrite(raw_sigs0[0])
    );

    Decoder ud1(
        .inst(dual_inst[63:32]),
        .opcode(raw_opc1),
        .func(raw_func1),
        .imm(raw_imm1),
        .rd(raw_regs1[4:0]),
        .rj(raw_regs1[9:5]),
        .rk(raw_regs1[14:10]),
        .memRead(raw_sigs1[2]),
        .memWrite(raw_sigs1[1]),
        .branch(raw_sigs1[3]),
        .jump(raw_sigs1[4]),
        .regWrite(raw_sigs1[0])
    );

    // ============================================================
    // 冲突检测 (需串行化的场景)
    // ============================================================
    wire raw_ls0 = raw_sigs0[2] || raw_sigs0[1];   // memRead | memWrite
    wire raw_ls1 = raw_sigs1[2] || raw_sigs1[1];
    wire raw_br0 = raw_sigs0[3];                    // branch
    wire raw_br1 = raw_sigs1[3];

    wire conflict_ls = raw_ls0 && raw_ls1;          // 双 LS → 只有槽1能跑
    wire conflict_br = raw_br0 && raw_br1;          // 双 Branch → 只有槽0能跑

    // ============================================================
    // 输出路由: raw → 实际输出
    // ============================================================
    always @(*) begin
        if (conflict_ls) begin
            // 槽0 的 LS 指令搬到槽1, 槽0 插入 NOP
            {opc0, func0, sigs0, imm0, regs0, nop0} =
                {10'b0,  7'b0,  5'b0,  32'b0, 15'b0, 1'b1};
            {opc1, func1, sigs1, imm1, regs1, nop1} =
                {raw_opc0, raw_func0, raw_sigs0, raw_imm0, raw_regs0, 1'b0};
        end
        else if (conflict_br) begin
            // 槽0 的 Branch 保留, 槽1 插入 NOP
            {opc0, func0, sigs0, imm0, regs0, nop0} =
                {raw_opc0, raw_func0, raw_sigs0, raw_imm0, raw_regs0, 1'b0};
            {opc1, func1, sigs1, imm1, regs1, nop1} =
                {10'b0,  7'b0,  5'b0,  32'b0, 15'b0, 1'b1};
        end
        else begin
            // 正常双发射
            {opc0, func0, sigs0, imm0, regs0, nop0} =
                {raw_opc0, raw_func0, raw_sigs0, raw_imm0, raw_regs0, 1'b0};
            {opc1, func1, sigs1, imm1, regs1, nop1} =
                {raw_opc1, raw_func1, raw_sigs1, raw_imm1, raw_regs1, 1'b0};
        end
    end

endmodule