//处理寄存器解码和发射逻辑
//解码-->识别依赖-->槽位分配-->输出各槽信号
//              |->指令nop标记-->传出

//为免混淆，称指令高位(高pc)低位(低pc)，最终决定发射的结果为槽1槽0
//输出的带0的信号即发射到槽0的指令，不过，槽0未必是低pc的那条指令
module ID_Stage(
    input clk,
    input [63:0] dual_inst,
    input [31:0] pc_low,          // 当前双指令中低 PC (供 ADEF 检测)
    input plv0,
    input int_pending,            // 中断待处理 (优先级高于一切异常)
    input stall,                  // 流水线暂停
    output [9:0] opc0, opc1,        // opcode
    output [6:0] func0, func1,      // func
    output [31:0] imm0, imm1,
    output [14:0] regs0, regs1,     // {rk, rj, rd}
    output [6:0] sigs0, sigs1,      // |valu[6]|jump[5]|branch[4]|memRead[3]|memWrite[2]|regWrite[1]|high[0]|
    output [16:0] csrBus0, csrBus1,  // |csreg[13:0]|xchg|csrwr|csrrd|
    output        evalid0, evalid1,
    output [5:0]  ecode0, ecode1,
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
    wire [16:0] csr0, csr1;
    wire        evalid0_raw, evalid1_raw;
    wire [5:0]  ecode0_raw, ecode1_raw;

    //这里的信号记作0,1，实际上，它们指的是低位和高位指令解码出的结果，不一定是槽0槽1最终发射的信号
    Decoder ud0(
        .inst(dual_inst[31:0]),
        .plv0(plv0),
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
        .valu(valu0),
        .csrBus(csr0),
        .evalid_in(1'b0),       // 低位指令无上游 ADEF
        .ecode_in(6'b0),
        .evalid(evalid0_raw),
        .ecode(ecode0_raw)
    );

    Decoder ud1(
        .inst(dual_inst[63:32]),
        .plv0(plv0),
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
        .valu(valu1),
        .csrBus(csr1),
        .evalid_in(1'b0),
        .ecode_in(6'b0),
        .evalid(evalid1_raw),
        .ecode(ecode1_raw)
    );

    // ============================================================
    // ADEF 检测: PC 未 4 字节对齐
    // ============================================================
    wire adef = (pc_low[1:0] != 2'b0);

    // ============================================================
    // 冲突检测信号
    // ============================================================
    wire ls0 = memRead0 || memWrite0;
    wire ls1 = memRead1 || memWrite1;
    wire br0 = jump0 || branch0 || (|csr0[2:1]);
    wire br1 = jump1 || branch1 || (|csr1[2:1]);

    wire conflict_ls  = ls0 && ls1;          // 双 LS → 只有槽1能跑, 移槽0→槽1
    wire conflict_br  = br0 && br1;          // 双 槽0独占 → 只有槽0能跑, 废弃槽1
    wire swap_ls      = ls0 && !ls1;          // 槽0=LS, 槽1≠LS → 交换(LS→槽1)
    wire swap_br      = br1 && !br0;          // 槽1=槽0独占, 槽0无 → 交换(→槽0)

    // ============================================================
    // 上一拍发射记录传递
    // ============================================================

    //上一拍槽1发射的指令的信息，记作9
    //仅在上一拍为load时，这一拍才有可能需要停顿
    reg load9;
    reg [4:0] rd9;

    always @(posedge clk) begin
        if(stall)begin
            load9<=load9;
            rd9<=rd9;
        end
        else if(load0&&!nopl)begin
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

    //store 也需检测依赖, MEM 级转发已去除 memRdata_ls_wb 组合路径
    wire dep10 = vrd0 &&
        (vrj1 && rj1==rd0 || vrk1 && rk1==rd0);//高位指令依赖低位指令结果
    wire dep09 = load9 &&
        (rd9==rj0 && vrj0 || rd9==rk0 && vrk0);//这一拍低位依赖上一拍load的结果
    wire dep19 = load9 &&
        (rd9==rj1 && vrj1 || rd9==rk1 && vrk1);//这一拍高位依赖上一拍load的结果

    //某条指令stall的方式就是将这条指令的nop置1
    //这样这一拍就不会发射这条指令，到了下一拍，inst_controll会取出未发射的内容，再次处理这条指令

    assign nopl = dep09;
    assign noph = dep10 || dep19 || conflict_ls || conflict_br;

    reg [87:0] bus0,bus1;
    wire [86:0] busl = {raw_opc0,raw_func0,raw_imm0,rk0,rj0,rd0,csr0,valu0,jump0,branch0,memRead0,memWrite0,regWrite0};
    wire [86:0] bush = {raw_opc1,raw_func1,raw_imm1,rk1,rj1,rd1,csr1,valu1,jump1,branch1,memRead1,memWrite1,regWrite1};


    always @(*) begin
        if(nopl || int_pending || adef)begin
            bus0 = 88'b0;
            bus1 = 88'b0;
        end
        else if(noph)begin
            if(ls0)begin
                bus0 = 88'b0;
                bus1 = {busl,1'b0};
            end
            else begin
                bus0 = {busl,1'b0};
                bus1 = 88'b0;
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

    assign {opc0,func0,imm0,regs0,csrBus0,sigs0} = bus0;
    assign {opc1,func1,imm1,regs1,csrBus1,sigs1} = bus1;

    // ============================================================
    // 异常信号赋值 (与 bus 独立, 复用 swap/stall 逻辑)
    // ============================================================
    reg        evalid0_out, evalid1_out;
    reg [5:0]  ecode0_out, ecode1_out;

    always @(*) begin
        if(nopl)begin
            evalid0_out = 1'b0; ecode0_out = 6'b0;
            evalid1_out = 1'b0; ecode1_out = 6'b0;
        end
        else if(int_pending)begin
            evalid0_out = 1'b1; ecode0_out = 6'h00;
            evalid1_out = 1'b1; ecode1_out = 6'h00;
        end
        else if(adef)begin
            evalid0_out = 1'b1; ecode0_out = 6'h08;
            evalid1_out = 1'b1; ecode1_out = 6'h08;
        end
        else if(noph)begin
            if(ls0)begin
                evalid0_out = 1'b0; ecode0_out = 6'b0;
                evalid1_out = evalid0_raw; ecode1_out = ecode0_raw;
            end
            else begin
                evalid0_out = evalid0_raw; ecode0_out = ecode0_raw;
                evalid1_out = 1'b0; ecode1_out = 6'b0;
            end
        end
        else begin
            if(swap_ls || swap_br)begin
                evalid0_out = evalid1_raw; ecode0_out = ecode1_raw;
                evalid1_out = evalid0_raw; ecode1_out = ecode0_raw;
            end
            else begin
                evalid0_out = evalid0_raw; ecode0_out = ecode0_raw;
                evalid1_out = evalid1_raw; ecode1_out = ecode1_raw;
            end
        end
    end

    assign evalid0 = evalid0_out;
    assign evalid1 = evalid1_out;
    assign ecode0  = ecode0_out;
    assign ecode1  = ecode1_out;
    

endmodule