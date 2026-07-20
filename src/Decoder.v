//不考虑浮点运算，暂不考虑特权指令等
module Decoder (
    input wire [31:0] inst,
    output wire [9:0] opcode,
    output wire [6:0] func,
    output reg [31:0] imm,
    // 寄存器地址 并非直接解码出的地址，rd为目标寄存器，rj,rk为数据源寄存器
    output wire [4:0] rd,
    output wire [4:0] rj,
    output wire [4:0] rk,
    output wire valid_rd,            // rd 字段有效 (指令写回 rd)
    output wire valid_rj,            // rj 字段有效 (指令用到 rj)
    output wire valid_rk,            // rk 字段有效 (指令用到 rk)
    output wire memRead, memWrite,
    output wire regWrite,
    output wire branch, //需要比较的跳转
    output wire jump,   // branch以外的跳转
    output wire alu,
    output wire load,
    output wire store,
    output wire valu           // EX结果会被写入寄存器 (≠load, load的WB数据来自MEM)
);

    // ============================================================
    // 指令字段提取
    // ============================================================
    // LA32 9种编码格式中，公共操作码位于高位：
    //   3R-type:   opcode(17) | rk(5) | rj(5) | rd(5)
    //   2RI12-type: opcode(10) | imm12(12) | rj(5) | rd(5)
    //   其他格式类似，操作码总是在最高位区域

    assign opcode = inst[31:22];
    assign func   = inst[21:15];

    // ============================================================
    // 指令类型检测
    // ============================================================

    // 特殊立即数ALU (1RI21型)
    wire isLU12IW    = (inst[31:26] == 6'b000101);
    wire isPCADDU12I = (inst[31:26] == 6'b000111);
    
    // I型ALU (2RI12格式: SLTI/SLTUI/ADDI.W/ANDI/ORI/XORI)
    //   31:26 = 000000, 且 25:22 != 0000 以排除 3R 型
    wire isShiftImm = (inst[31:22] == 10'b0000000001);
    wire isALUimm   = (inst[31:26] == 6'b000000) && (inst[25:22] != 4'b0000)
                   && !isShiftImm;
    wire isANDI     = (inst[31:22] == 10'b0000001101);
    wire isORI      = (inst[31:22] == 10'b0000001110);
    wire isXORI     = (inst[31:22] == 10'b0000001111);
    wire isLogicImm = isANDI || isORI || isXORI;

    // 访存指令 (opcode[31:27] = 00101)
    //   编码: inst[26]=0→整数, inst[24]=0→load, inst[24]=1→store
    //         inst[26]=1→浮点(暂不实现)
    wire isLoad  = (inst[31:27] == 5'b00101) && !inst[26] && !inst[24];
    wire isStore = (inst[31:27] == 5'b00101) && !inst[26] &&  inst[24];

    // 浮点访存 (opcode[31:27] = 00101, inst[26:25] = 11)
    // wire isFPStore = (inst[31:27] == 5'b00101) && (inst[26:25] == 2'b11) && inst[24];
    // wire isFPLoad  = (inst[31:27] == 5'b00101) && (inst[26:25] == 2'b11) && !inst[24];

    // 分支: opcode[31:26] 落在 010110 ~ 011011 范围内
    wire isBranch  = (inst[31:27] == 5'b01011) || (inst[31:27] == 5'b01100) || (inst[31:27] == 5'b01101);

    // 无条件跳转
    wire isB      = (inst[31:26] == 6'b010100);
    wire isBL     = (inst[31:26] == 6'b010101);
    wire isJIRL   = (inst[31:26] == 6'b010011);

    // 浮点条件分支
    // wire isBCEQZ  = (inst[31:26] == 6'b010010) && !inst[5];
    // wire isBCNEZ  = (inst[31:26] == 6'b010010) && inst[5];

    // ============================================================
    // 寄存器地址提取
    // ============================================================
    // rj 固定在 inst[9:5]
    // rd:  store/branch无写回目标置0; BL写r1; 其余用 inst[4:0]
    // rk:  store/branch的数据源在 inst[4:0]; 其余 3R 型用 inst[14:10]
    wire rdIsSrc = isStore || isBranch;       // store/branch 把 rd 当源操作数
    assign rj = inst[9:5];
    assign rd = isBL      ? 5'd1         :
                rdIsSrc   ? 5'b0         :
                            inst[4:0];
    assign rk = rdIsSrc   ? inst[4:0]    :
                            inst[14:10];

    // ============================================================
    // 立即数生成
    // ============================================================
    // 根据指令类型从不同位域拼接并做符号/零扩展
    always @(*) begin
        if (isLU12IW || isPCADDU12I) begin
            // 1RI21: si20 << 12
            imm = {inst[24:5], 12'b0};
        end
        else if (isB || isBL) begin
            // I26: offs26 << 2, 符号扩展
            imm = {{4{inst[9]}}, inst[9:0], inst[25:10], 2'b0};
        end
        else if (isBranch || isJIRL) begin
            // 2RI16: offs16 << 2, 符号扩展
            imm = {{14{inst[25]}}, inst[25:10], 2'b0};
        end
        else if (isLogicImm) begin
            // ANDI/ORI/XORI: ui12 零扩展
            imm = {20'b0, inst[21:10]};
        end
        else if (isALUimm || isLoad || isStore) begin
            // SLTI/SLTUI/ADDI.W/LD/ST: si12 符号扩展
            imm = {{20{inst[21]}}, inst[21:10]};
        end
        else if (isShiftImm) begin
            // SLLI.W/SRLI.W/SRAI.W: ui5 = inst[14:10], 零扩展
            imm = {27'b0, inst[14:10]};
        end
        else begin
            // 3R型 或 其他: 无立即数, 置0
            imm = 32'b0;
        end
    end

    // ============================================================
    // 控制信号
    // ============================================================

    // 3R型ALU (排除BREAK/SYSCALL)
    wire isBREAK   = (inst[31:15] == 17'b00000000001010100);
    wire isSYSCALL = (inst[31:15] == 17'b00000000001010110);
    wire is3R_ALU  = (inst[31:22] == 10'b0000000000) && !isBREAK && !isSYSCALL;

    assign memRead  = isLoad;
    assign memWrite = isStore;

    assign branch = isBranch;
    assign jump   = isB || isBL || isJIRL;

    assign alu   = is3R_ALU || isALUimm || isShiftImm
                || isLU12IW || isPCADDU12I;
    assign load  = isLoad;
    assign store = isStore;
    assign valu  = regWrite && !isLoad;               // EX结果写回 (load的WB数据来自MEM)

    assign regWrite = is3R_ALU || isALUimm || isShiftImm  // ALU运算写回
                   || isLoad                               // 加载写回
                   || isBL || isJIRL                       // 链接跳转写回
                   || isLU12IW || isPCADDU12I;             // 立即数装载

    // ============================================================
    // 寄存器字段有效性
    // ============================================================
    // 若本条指令中，rd是目标，rj,rk是数据源，则对应valid信号为真
    assign valid_rd = regWrite && (rd != 5'b0);                // BL已设rd=1, 无需排除
    assign valid_rj = !(isLU12IW || isPCADDU12I  // 1RI21: inst[9:5]属si20
                     || isB || isBL);            // I26:  inst[9:5]属offs26
    assign valid_rk = is3R_ALU || rdIsSrc;       // 3R型 或 rd作数据源(store/branch)

endmodule