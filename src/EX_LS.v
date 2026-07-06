module EX_LS(
    input clk,
    input en,
    input memRead, memWrite,     // 来自 Decoder 的访存控制
    input [9:0] opcode,          // inst[31:22]
    input [6:0] func,            // inst[21:15]
    input [31:0] reg1,           // rj (基址 或 ALU操作数1)
    input [31:0] reg2,           // rk 或立即数 (ALU操作数2)
    input [31:0] store_data,     // rd 值 (仅 store 时有效)
    input [31:0] imm,            // 解码后立即数
    output reg [31:0] alu_result,    // ALU结果 / 访存地址
    output mem_we,               // 存储器写使能
    output [1:0] mem_size,       // 00=byte, 01=halfword, 10=word
    output [31:0] mem_wdata      // 写入存储器的数据
);

    // ============================================================
    // 指令子类型检测
    //   opcode[9:4] = inst[31:26]   opcode[3:0] = inst[25:22]
    //   opcode[9:3] = inst[31:25]
    // ============================================================

    // 3R型 ALU: 完整 opcode = 10'h000
    wire is3R     = (opcode == 10'h000);

    // I型 ALU
    wire isImmALU = (opcode[9:4] == 6'h00) && (opcode != 10'h000)
                                              && (opcode != 10'h004);

    // 移位立即数
    wire isShiftI = (opcode == 10'h004);

    // 特殊 1RI21
    wire isLU12I  = (opcode[9:3] == 7'b0001010);
    wire isPCADDU = (opcode[9:3] == 7'b0001110);

    // ============================================================
    // 操作数2 选择: 3R 用 reg2(rk), 其余用 imm
    // ============================================================
    wire [31:0] operand2;
    assign operand2 = is3R ? reg2 : imm;

    // ============================================================
    // 3R型 ALU 功能码
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
    // I型 ALU 操作
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
    // 访存相关
    // ============================================================
    // 访存地址 = rj + SignExt(si12) = reg1 + imm
    wire [31:0] mem_addr;
    assign mem_addr = reg1 + imm;

    // 访存宽度 (所有 LS 指令的 opcode[1:0] = inst[23:22])
    //   00 = byte, 01 = halfword, 10 = word
    assign mem_size = opcode[1:0];

    assign mem_we    = memWrite;
    assign mem_wdata = store_data;

    // ============================================================
    // ALU 主逻辑 (与 EX_ALU 相同，但无分支/跳转)
    // ============================================================
    reg  [63:0] product;

    always @(*) begin
        if (!en) begin
            alu_result = 32'b0;
        end
        // ── Load/Store: 地址 = reg1 + imm ──
        else if (memRead || memWrite) begin
            alu_result = mem_addr;
        end
        // ── 特殊立即数 ──
        else if (isLU12I) begin
            alu_result = imm;
        end
        else if (isPCADDU) begin
            alu_result = reg1 + imm;    // 注意: PCADDU12I 用 pc, 但 slot1 无 pc
        end
        // ── ALU 运算 ──
        else begin
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

            // --- 移位 ---
            else if (fSLL)   alu_result = reg1 << operand2[4:0];
            else if (sSLLI)  alu_result = reg1 << operand2[4:0];
            else if (fSRL)   alu_result = reg1 >> operand2[4:0];
            else if (sSRLI)  alu_result = reg1 >> operand2[4:0];
            else if (fSRA)   alu_result = $signed(reg1) >>> operand2[4:0];
            else if (sSRAI)  alu_result = $signed(reg1) >>> operand2[4:0];

            // --- 乘法 ---
            else if (fMUL) begin
                product    = $signed(reg1) * $signed(operand2);
                alu_result = product[31:0];
            end
            else if (fMULH) begin
                product    = $signed(reg1) * $signed(operand2);
                alu_result = product[63:32];
            end
            else if (fMULHU) begin
                product    = reg1 * operand2;
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

endmodule