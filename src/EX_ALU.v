module EX_ALU(
    input clk,
    input en,
    input branch, jump,
    input [9:0] opcode,          // inst[31:22]
    input [6:0] func,            // inst[21:15]
    input [31:0] reg1, reg2,     // 寄存器值 (rj, rk/rd)
    input [31:0] imm,            // 解码后立即数
    input [31:0] pc,             // 当前指令PC
    input [31:0] csr_read,       // CSR 读出的旧值
    input [16:0] csrBus,         // CSR 控制总线
    input [31:0] era, prmd,      // 异常返回信息
    output reg [31:0] alu_result,
    output [31:0] csr_result,    // 将写入 CSR 的值
    output jump_taken,       // branch成功 或 jump 时为 1
    output reg branch_taken, //仅当branch判定成功时为1
    output reg [31:0] jump_addr
);

    // ============================================================
    // 指令子类型检测
    //   opcode[9:4] = inst[31:26]   opcode[3:0] = inst[25:22]
    //   opcode[9:3] = inst[31:25]
    // ============================================================

    // 3R型 ALU: 完整 opcode = 10'h000
    wire is3R     = (opcode == 10'h000);

    // I型 ALU (SLTI/SLTUI/ADDI.W/ANDI/ORI/XORI):
    //   inst[31:26]=6'h00 且不是 3R(全0) 也不是移位立即数(004)
    wire isImmALU = (opcode[9:4] == 6'h00) && (opcode != 10'h000)
                                              && (opcode != 10'h004);

    // 移位立即数 (SLLI/SRLI/SRAI): opcode = 10'h004
    wire isShiftI = (opcode == 10'h004);

    // 特殊 1RI21 型: inst[31:25] 固定, 低 3bit 属于 si20
    wire isLU12I  = (opcode[9:3] == 7'b0001010);
    wire isPCADDU = (opcode[9:3] == 7'b0001110);

    // ============================================================
    // 操作数2 选择: 3R 用 reg2(rk), 其余用 imm
    // ============================================================
    wire [31:0] operand2;
    assign operand2 = is3R ? reg2 : imm;

    // ============================================================
    // 分支比较类型 (inst[31:26] = opcode[9:4])
    // ============================================================
    wire isBEQ  = (opcode[9:4] == 6'h16);
    wire isBNE  = (opcode[9:4] == 6'h17);
    wire isBLT  = (opcode[9:4] == 6'h18);
    wire isBGE  = (opcode[9:4] == 6'h19);
    wire isBLTU = (opcode[9:4] == 6'h1A);
    wire isBGEU = (opcode[9:4] == 6'h1B);

    // ============================================================
    // 跳转类型
    // ============================================================
    wire isB    = (opcode[9:4] == 6'h14);
    wire isBL   = (opcode[9:4] == 6'h15);
    wire isJIRL = (opcode[9:4] == 6'h13);

    // ============================================================
    // 3R型 ALU 功能码 (func[6:0] = inst[21:15])
    // ============================================================
    wire fADD   = (func == 7'h20);
    wire fSUB   = (func == 7'h22);
    wire fSLT   = (func == 7'h24);
    wire fSLTU  = (func == 7'h25);
    wire fNOR   = (func == 7'h28);
    wire fAND   = (func == 7'h29);
    wire fOR    = (func == 7'h2A);
    wire fXOR   = (func == 7'h2B);
    wire fSLL   = (func == 7'h2E);
    wire fSRL   = (func == 7'h2F);
    wire fSRA   = (func == 7'h30);
    wire fMUL   = (func == 7'h38);
    wire fMULH  = (func == 7'h39);
    wire fMULHU = (func == 7'h3A);
    wire fDIV   = (func == 7'h40);
    wire fMOD   = (func == 7'h41);
    wire fDIVU  = (func == 7'h42);
    wire fMODU  = (func == 7'h43);

    // ============================================================
    // I型 ALU 操作 (opcode 低 4bit 区分)
    // ============================================================
    wire iADDI  = (opcode == 10'h00A);   // 0000001010
    wire iANDI  = (opcode == 10'h00D);   // 0000001101
    wire iORI   = (opcode == 10'h00E);   // 0000001110
    wire iXORI  = (opcode == 10'h00F);   // 0000001111
    wire iSLTI  = (opcode == 10'h008);   // 0000001000
    wire iSLTUI = (opcode == 10'h009);   // 0000001001

    // 移位立即数子类型
    wire sSLLI  = (func == 7'h01);
    wire sSRLI  = (func == 7'h09);
    wire sSRAI  = (func == 7'h11);

    // ============================================================
    // 主逻辑
    // ============================================================
    // 64-bit 中间结果 (乘除法用)
    reg [63:0] product;
    reg [31:0] quotient, remainder;

    always @(*) begin
        if (!en) begin
            alu_result = 32'b0;
            branch_taken = 1'b0;
            jump_addr  = 32'b0;
        end
        // ── ERTN: 跳转至 ERA ──
        else if (isERTN) begin
            alu_result   = 32'b0;
            branch_taken = 1'b0;
            jump_addr    = era;
        end
        // ── CSR 操作 (优先级最高, 读回值写入 GPR) ──
        else if (is_csr) begin
            alu_result   = csr_read;
            branch_taken = 1'b0;
            jump_addr    = 32'b0;
        end
        // ── 跳转 ──
        else if (jump) begin
            branch_taken = 1'b0;
            if (isB) begin
                // B: 无条件跳, 无写回
                alu_result = 32'b0;
                jump_addr  = pc + imm;
            end
            else if (isBL) begin
                // BL: 无条件跳, r1 ← pc+4
                alu_result = pc + 32'd4;
                jump_addr  = pc + imm;
            end
            else begin  // isJIRL
                // JIRL: rd ← pc+4, 跳转到 rj+offs
                alu_result = pc + 32'd4;
                jump_addr  = reg1 + imm;
            end
        end
        // ── 分支 ──
        else if (branch) begin
            alu_result = 32'b0;
            jump_addr  = pc + imm;

            if (isBEQ)
                branch_taken = (reg1 == reg2);
            else if (isBNE)
                branch_taken = (reg1 != reg2);
            else if (isBLT)
                branch_taken = ($signed(reg1) < $signed(reg2));
            else if (isBGE)
                branch_taken = ($signed(reg1) >= $signed(reg2));
            else if (isBLTU)
                branch_taken = (reg1 < reg2);
            else if (isBGEU)
                branch_taken = (reg1 >= reg2);
            else
                branch_taken = 1'b0;
        end
        // ── 特殊立即数 ──
        else if (isLU12I) begin
            branch_taken = 1'b0;
            jump_addr  = 32'b0;
            alu_result = imm;               // {si20, 12'b0}
        end
        else if (isPCADDU) begin
            branch_taken = 1'b0;
            jump_addr  = 32'b0;
            alu_result = pc + imm;          // pc + {si20, 12'b0}
        end
        // ── ALU ──
        else begin
            branch_taken = 1'b0;
            jump_addr  = 32'b0;

            // --- 加法 ---
            if (fADD || iADDI)
                alu_result = reg1 + operand2;

            // --- 减法 ---
            else if (fSUB)
                alu_result = reg1 - operand2;

            // --- 有符号比较 ---
            else if (fSLT || iSLTI)
                alu_result = ($signed(reg1) < $signed(operand2)) ? 32'd1 : 32'd0;

            // --- 无符号比较 ---
            else if (fSLTU || iSLTUI)
                alu_result = (reg1 < operand2) ? 32'd1 : 32'd0;

            // --- 按位逻辑 ---
            else if (fNOR)
                alu_result = ~(reg1 | operand2);
            else if (fAND || iANDI)
                alu_result = reg1 & operand2;
            else if (fOR || iORI)
                alu_result = reg1 | operand2;
            else if (fXOR || iXORI)
                alu_result = reg1 ^ operand2;

            // --- 逻辑左移 (移位量取低5位) ---
            else if (fSLL)   alu_result = reg1 << operand2[4:0];
            else if (sSLLI)  alu_result = reg1 << operand2[4:0];

            // --- 逻辑右移 ---
            else if (fSRL)   alu_result = reg1 >> operand2[4:0];
            else if (sSRLI)  alu_result = reg1 >> operand2[4:0];

            // --- 算术右移 ---
            else if (fSRA)   alu_result = $signed(reg1) >>> operand2[4:0];
            else if (sSRAI)  alu_result = $signed(reg1) >>> operand2[4:0];

            // --- 乘法 ---
            else if (fMUL) begin
                product   = $signed(reg1) * $signed(operand2);
                alu_result = product[31:0];
            end
            else if (fMULH) begin
                product   = $signed(reg1) * $signed(operand2);
                alu_result = product[63:32];
            end
            else if (fMULHU) begin
                product   = reg1 * operand2;
                alu_result = product[63:32];
            end

            // --- 除法 / 取模 (除0返回0) ---
            else if (fDIV)
                alu_result = (operand2 != 0)
                    ? $signed(reg1) / $signed(operand2) : 32'd0;
            else if (fMOD)
                alu_result = (operand2 != 0)
                    ? $signed(reg1) % $signed(operand2) : 32'd0;
            else if (fDIVU)
                alu_result = (operand2 != 0)
                    ? reg1 / operand2 : 32'd0;
            else if (fMODU)
                alu_result = (operand2 != 0)
                    ? reg1 % operand2 : 32'd0;

            // --- 默认 ---
            else
                alu_result = 32'b0;
        end
    end

    assign jump_taken = jump || branch_taken;

    // ============================================================
    // ERTN 检测: jump=1, csrwr=1, addr=CRMD
    // ============================================================
    wire isERTN = jump && csrBus[1] && (csrBus[16:3] == 14'h0);

    // ============================================================
    // CSR 操作: csr_result = 将写入 CSR 的值
    //   CSRWR:  rd 值直写
    //   CSRXCHG: (rd & rj) | (旧CSR & ~rj), rj 为掩码
    // ============================================================
    wire is_csr    = |csrBus[2:0];
    wire is_csrwr  = csrBus[1];
    wire [31:0] csr_wdata_raw = isERTN   ? {29'b0, prmd[2], prmd[1:0]}  // PRMD→CRMD
                               : is_csrwr ? reg2
                               : (reg2 & reg1) | (csr_read & ~reg1);
    assign csr_result = (is_csr || isERTN) ? csr_wdata_raw : 32'b0;

endmodule