//处理寄存器解码和发射逻辑
//解码-->识别依赖-->槽位分配-->输出各槽信号
//              |->指令nop标记-->传出

//为免混淆，称指令高位(高pc)低位(低pc)，最终决定发射的结果为槽1槽0
//输出的带0的信号即发射到槽0的指令，不过，槽0未必是低pc的那条指令
module ID_Stage(
    input clk,
    input [63:0] dual_inst,
    output [9:0] opc0, opc1,        // opcode
    output [6:0] func0, func1,      // func
    output [31:0] imm0, imm1,
    output [14:0] regs0, regs1,     // {rk, rj, rd}//|valu[6]|jump[5]|branch[4]|memRead[3]|memWrite[2]|regWrite[1]|high[0]|
    output [6:0] sigs0, sigs1,
    output nopl, noph               //若低位(pc较小)的指令未发射，则nopl为1;noph代表高位，同理
);

    // ============================================================
    // Decoder 原始输出 (raw_*)
    // ============================================================
    wire [9:0]  raw_opc0, raw_opc1;
    wire [6:0]  raw_func0, raw_func1;
    wire [31:0] raw_imm0, raw_imm1;

    wire [4:0]  rd0, rj0, rk0, rd1, rj1, rk1;
    wire        memRead0,  memWrite0, branch0, jump0, regWrite0;
    wire        memRead1,  memWrite1, branch1, jump1, regWrite1;
    wire        alu0, load0, store0, valu0;
    wire        alu1, load1, store1, valu1;
    wire        vrd0, vrj0, vrk0;
    wire        vrd1, vrj1, vrk1;

    //这里的信号记作0,1，实际上，它们指的是低位和高位指令解码出的结果，不一定是槽0槽1最终发射的信号
    Decoder ud0(
        .inst(dual_inst[31:0]),
        .opcode(raw_opc0),
        .func(raw_func0),
        .imm(raw_imm0),
        .rd(rd0),
        .rj(rj0),
        .rk(rk0),
        .valid_rd(vrd0),
        .valid_rj(vrj0),
        .valid_rk(vrk0),
        .memRead(memRead0),
        .memWrite(memWrite0),
        .branch(branch0),
        .jump(jump0),
        .regWrite(regWrite0),
        .alu(alu0),
        .load(load0),
        .store(store0),
        .valu(valu0)
    );

    Decoder ud1(
        .inst(dual_inst[63:32]),
        .opcode(raw_opc1),
        .func(raw_func1),
        .imm(raw_imm1),
        .rd(rd1),
        .rj(rj1),
        .rk(rk1),
        .valid_rd(vrd1),
        .valid_rj(vrj1),
        .valid_rk(vrk1),
        .memRead(memRead1),
        .memWrite(memWrite1),
        .branch(branch1),
        .jump(jump1),
        .regWrite(regWrite1),
        .alu(alu1),
        .load(load1),
        .store(store1),
        .valu(valu1)
    );

    // ============================================================
    // 冲突检测信号
    // ============================================================
    wire ls0 = memRead0 || memWrite0;
    wire ls1 = memRead1 || memWrite1;
    wire br0 = jump0 || branch0;
    wire br1 = jump1 || branch1;

    wire conflict_ls = ls0 && ls1;          // 双 LS → 只有槽1能跑, 移槽0→槽1
    wire conflict_br = br0 && br1;          // 双 Branch → 只有槽0能跑, 废弃槽1
    wire swap_ls     = ls0 && !ls1;          // 槽0=LS, 槽1≠LS → 交换(LS→槽1)
    wire swap_br     = br1 && !br0;          // 槽1=BR, 槽0≠BR → 交换(BR→槽0)

    // ============================================================
    // 上一拍发射记录传递
    // ============================================================

    //上一拍槽1发射的指令的信息，记作9
    //仅在上一拍为load时，这一拍才有可能需要停顿
    reg load9;
    reg [4:0] rd9;

    always @(posedge clk) begin
        if(load0&&!nopl)begin
            load9<=1'b1;
            rd9<=rd0;
        end
        else if(load1&&!nopl&&!noph)begin
            load9<=1'b1;
            rd9<=rd1;
        end
        else begin
            load9<=1'b0;
            rd9<=5'b0;
        end
    end

    // ============================================================
    // 同拍依赖检测
    // ============================================================

    //排除store，因其MEM才使用数据，可forwarding
    wire dep10 = vrd0 && !store1 && 
        (vrj1 && rj1==rd0 || vrk1 && rk1==rd0);//高位指令依赖低位指令结果
    wire dep09 = load9 && !store0 && 
        (rd9==rj0 && vrj0 || rd9==rk0 && vrk0);//这一拍低位依赖上一拍load的结果
    wire dep19 = load9 && !store1 && 
        (rd9==rj1 && vrj1 || rd9==rk1 && vrk1);//这一拍高位依赖上一拍load的结果

    //某条指令stall的方式就是将这条指令的nop置1
    //这样这一拍就不会发射这条指令，到了下一拍，inst_controll会取出未发射的内容，再次处理这条指令

    assign nopl = dep09;
    assign noph = dep10 || dep19 || conflict_ls || conflict_br;

    reg [70:0] bus0,bus1;
    wire [69:0] busl = {raw_opc0,raw_func0,raw_imm0,rk0,rj0,rd0,valu0,jump0,branch0,memRead0,memWrite0,regWrite0};
    wire [69:0] bush = {raw_opc1,raw_func1,raw_imm1,rk1,rj1,rd1,valu1,jump1,branch1,memRead1,memWrite1,regWrite1};


    always @(*) begin
        if(nopl)begin
            bus0 = 71'b0;
            bus1 = 71'b0;
        end
        else if(noph)begin
            if(branch0)begin
                bus0 = {busl,1'b1};
                bus1 = 71'b0;
            end
            else begin
                bus0 = 71'b0;
                bus1 = {busl,1'b1};
            end
        end
        else begin
            //sigbus中的high信号，表示若双发射，当前槽是否pc更高
            if(swap_ls || swap_br)begin
                bus0 = {bush,1'b1};
                bus1 = {busl,1'b0};
            end
            else begin
                bus0 = {busl,1'b0};
                bus1 = {bush,1'b1};
            end
        end
    end

    assign {opc0,func0,imm0,regs0,sigs0} = bus0;
    assign {opc1,func1,imm1,regs1,sigs1} = bus1;
    

endmodule