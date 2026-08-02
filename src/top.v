//双发射，槽0可branch,槽1可load/store，二者都兼容alu
//bl?
module top #(
    parameter IMEM_FILE  = "imem_init.hex" ,
    parameter DMEM_FILE  = "dmem_init.hex"
)
(
    input           clk,
    input           rst_n,
    input  [ 7:0]   hwi,
    input           ipi,

    // ── AXI 总线 ──
    // read request
    output [ 3:0]   arid,
    output [31:0]   araddr,
    output [ 7:0]   arlen,
    output [ 2:0]   arsize,
    output [ 1:0]   arburst,
    output [ 1:0]   arlock,
    output [ 3:0]   arcache,
    output [ 2:0]   arprot,
    output          arvalid,
    input           arready,
    // read data
    input  [ 3:0]   rid,
    input  [31:0]   rdata,
    input  [ 1:0]   rresp,
    input           rlast,
    input           rvalid,
    output          rready,
    // write request
    output [ 3:0]   awid,
    output [31:0]   awaddr,
    output [ 7:0]   awlen,
    output [ 2:0]   awsize,
    output [ 1:0]   awburst,
    output [ 1:0]   awlock,
    output [ 3:0]   awcache,
    output [ 2:0]   awprot,
    output          awvalid,
    input           awready,
    // write data
    output [ 3:0]   wid,
    output [31:0]   wdata,
    output [ 3:0]   wstrb,
    output          wlast,
    output          wvalid,
    input           wready,
    // write response
    input  [ 3:0]   bid,
    input  [ 1:0]   bresp,
    input           bvalid,
    output          bready
);

localparam NOP = 32'h03400000;

// ── PC & IF 信号 ──
wire [31:0] pc_next;
wire [63:0] dual_inst_raw;
wire [63:0] dual_inst;
wire        if_stall;                   // iCache stall → pipeline

// ── MEM 访存 & stall ──
wire        dcache_stall;               // dCache stall → pipeline
wire [ 3:0] mem_write_strb;            // 字节写使能

// ── cache → axi_bridge (AXI burst) ──
wire        i_arvalid,  d_arvalid;
wire [31:0] i_araddr,   d_araddr;
wire [ 7:0] i_arlen,    d_arlen;
wire [ 2:0] i_arsize,   d_arsize;
wire        i_arready,  d_arready;
wire        i_rvalid,   d_rvalid;
wire [31:0] i_rdata,    d_rdata;
wire [ 1:0] i_rresp,    d_rresp;
wire        i_rlast,    d_rlast;
wire        i_rready,   d_rready;
// dCache write
wire        d_awvalid;
wire [31:0] d_awaddr;
wire [ 7:0] d_awlen;
wire [ 2:0] d_awsize;
wire        d_awready;
wire        d_wvalid;
wire [31:0] d_wdata;
wire [ 3:0] d_wstrb;
wire        d_wlast;
wire        d_wready;
wire        d_bvalid;
wire [ 1:0] d_bresp;
wire [ 3:0] d_bid;
wire        d_bready;


// ID Stage 输出
wire [9:0]  opcode_br_id, opcode_ls_id;
wire [6:0]  func_br_id, func_ls_id;
wire [6:0]  sigBus_br_id, sigBus_ls_id;
wire [31:0] imm_br_id, imm_ls_id;
wire [14:0] regAddr_br_id, regAddr_ls_id;
wire [16:0] csrBus_br_id, csrBus_ls_id;
wire        evalid_br_id, evalid_ls_id;
wire [5:0]  ecode_br_id, ecode_ls_id;
wire        nopl, noph;
wire        plv0;                     // 来自 CSR, 当前特权等级
wire [31:0] csr_rdata_br, csr_rdata_ls;  // CSR 读回数据
wire [31:0] eentry_val;                    // 异常入口地址
wire [31:0] era_val, prmd_val;             // 异常返回信息 (供 ERTN)
wire        int_pending;                   // 中断待处理
wire        da, pg;                         // CRMD 翻译模式
wire [1:0]  plv;                            // 当前特权等级
wire [31:0] dmw0_val, dmw1_val;             // DMW 窗口

// ── Regs 写端口 (WB 级驱动) ──
reg  [4:0]  regAddr_rd_br_wb, regAddr_rd_ls_wb;       // 写回地址 (rd)
reg  [31:0] regData_br_wb;                   // 写回数据 槽0
wire [31:0] regData_ls_wb;                   // 写回数据 槽1 (mux 输出, 保持 wire)
reg         regWrite_br_wb, regWrite_ls_wb;             // 写使能
reg         high_br_wb, high_ls_wb;                     // PC高低 (供前递仲裁)

// ── Regs 读端口数据 ──
wire [31:0] regData_rj_br_id, regData_rk_br_id;           // 槽0 读出 (rj, rk)
wire [31:0] regData_rj_ls_id, regData_rk_ls_id;           // 槽1 读出 (rj, rk)

// ============================================================
// EX → MEM 流水线寄存器
//   slot 1: 访存控制 + WB 穿越信号
//   slot 0: 中继气泡 (与 slot 1 同步到达 WB)
// ============================================================

// ── 槽0 中继: EX → relay → WB (NBA 逐级传递, 2 拍对齐槽1) ──
reg [4:0]  regAddr_rd_br_mem;
reg [31:0] aluResult_br_mem;
reg [6:0]  sigBus_br_mem;
reg [16:0] csrBus_br_mem;                              // CSR 总线 (槽0中继)
reg [31:0] csrResult_br_mem;                           // CSR 写入数据 (槽0中继)
reg        evalid_br_mem, evalid_ls_mem;               // 异常
reg [5:0]  ecode_br_mem, ecode_ls_mem;
reg [31:0] pc_low_br_mem, pc_low_ls_mem;               // 指令 PC

// ── 槽1 EX→MEM (访存 + WB 穿越) ──
reg        memWrite_ls_mem;
reg [1:0]  memSize_ls_mem;
reg [31:0] memAddr_ls_mem;
reg [31:0] memWdata_ls_mem;
// WB 穿越信号
reg [4:0]  regAddr_rd_ls_mem;
reg [4:0]  regAddr_rk_ls_mem;            // store数据源寄存器 (供MEM前递比较)
reg [31:0] aluResult_ls_mem;
reg [6:0]  sigBus_ls_mem;
reg        signExt_ls_mem;               // load 符号扩展标志
reg [16:0] csrBus_ls_mem;                              // CSR 总线 (槽1)

// ── WB 级 CSR 总线 (槽0/槽1) ──
reg [16:0] csrBus_br_wb, csrBus_ls_wb;
reg [31:0] csrResult_br_wb;                            // CSR 写入数据 (槽0 WB)
reg        evalid_br_wb, evalid_ls_wb;                 // 异常
reg [5:0]  ecode_br_wb, ecode_ls_wb;
reg [31:0] pc_low_br_wb, pc_low_ls_wb;                 // 指令 PC


wire        stall;                                     // 流水线暂停


// ── 地址翻译 ──
wire [31:0] if_pa, mem_pa;
mmu u_mmu (
    .if_va (pc_next),
    .mem_va(memAddr_ls_mem),
    .plv   (plv), .da(da), .pg(pg),
    .dmw0  (dmw0_val), .dmw1(dmw1_val),
    .if_pa (if_pa),
    .mem_pa(mem_pa)
);

IF_Stage uif (
    .clk        (clk),
    .rst_n      (rst_n),
    .pc         (if_pa),
    .dual_inst  (dual_inst_raw),
    .if_stall   (if_stall),
    .arvalid    (i_arvalid),
    .araddr     (i_araddr),
    .arlen      (i_arlen),
    .arsize     (i_arsize),
    .arready    (i_arready),
    .rvalid     (i_rvalid),
    .rdata      (i_rdata),
    .rresp      (i_rresp),
    .rlast      (i_rlast),
    .rready     (i_rready)
);

wire [31:0] pc_low_id;


ID_Stage uid(
    .clk(clk),
    .dual_inst(dual_inst),
    .pc_low(pc_low_id),
    .plv0(plv0),                  // 来自 CSR 的特权等级
    .int_pending(int_pending),    // 来自 CSR 的中断信号
    .stall(stall),                // 流水线暂停
    .opc0(opcode_br_id),
    .opc1(opcode_ls_id),
    .func0(func_br_id),
    .func1(func_ls_id),
    .sigs0(sigBus_br_id),
    .sigs1(sigBus_ls_id),
    .imm0(imm_br_id),
    .imm1(imm_ls_id),
    .regs0(regAddr_br_id),
    .regs1(regAddr_ls_id),
    .csrBus0(csrBus_br_id),
    .csrBus1(csrBus_ls_id),
    .evalid0(evalid_br_id),
    .evalid1(evalid_ls_id),
    .ecode0(ecode_br_id),
    .ecode1(ecode_ls_id),
    .nopl(nopl),
    .noph(noph)
);



// ============================================================
// ID -> EX 流水线寄存器
// ============================================================
reg [9:0]  opcode_br_ex, opcode_ls_ex;
reg [6:0]  func_br_ex, func_ls_ex;
reg [6:0]  sigBus_br_ex, sigBus_ls_ex;

reg [4:0]  regAddr_rd_br_ex, regAddr_rd_ls_ex;            // 目标寄存器地址 (仅保留 rd, 供 WB 写回)
reg [4:0]  regAddr_rj_br_ex, regAddr_rk_br_ex;            // 源寄存器地址 (供前递比较)
reg [4:0]  regAddr_rj_ls_ex, regAddr_rk_ls_ex;
reg [31:0] imm_br_ex, imm_ls_ex;
reg [31:0] pc_low_ex;
reg [31:0] regData_rj_br_ex, regData_rk_br_ex;          // 槽0 寄存器值 (流水线后)
reg [31:0] regData_rj_ls_ex, regData2_ls_ex;          // 槽1 寄存器值 (流水线后: rj, rk/rd*)
reg [16:0] csrBus_br_ex, csrBus_ls_ex;                 // CSR 总线
reg         evalid_br_ex, evalid_ls_ex;                 // 异常有效
reg [5:0]   ecode_br_ex, ecode_ls_ex;                   // 异常编码
wire        branch_taken_br_ex;           // EX_ALU 分支判定结果
wire        flush_id, flush_ex, flush_ls_mem;  // 流水线冲刷
wire        flush_br_mem, flush_br_wb, flush_ls_wb;  // 异常冲刷

// ── EX 级信号 ──
wire [31:0] aluResult_br_ex;              // 槽0 ALU 结果 (csr 读回值也走此线写入 GPR)
wire [31:0] csrResult_br_ex;              // 槽0 CSR 写入数据 (将写入 CSR)
wire        jumpTaken_br_ex;
wire [31:0] jumpAddr_br_ex;

wire [31:0] aluResult_ls_ex;            // 访存地址 / ALU 结果
wire        memWrite_ls_ex;                // 存储器写使能
wire [1:0]  memSize_ls_ex;              // 访存宽度: 00=byte, 01=half, 10=word
wire [31:0] memWdata_ls_ex;             // 写入存储器的数据

// ── ALE 检测: 访存且地址未对齐 ──
wire ale = (sigBus_ls_ex[3] || sigBus_ls_ex[2]) && (
    (memSize_ls_ex == 2'b01 && aluResult_ls_ex[0]) ||
    (memSize_ls_ex == 2'b10 && |aluResult_ls_ex[1:0])
);

// 访存门控: 异常指令不产生副作用
wire gate_ls = ale || evalid_ls_ex || (evalid_br_ex && sigBus_ls_ex[0]);


always @(posedge clk) begin
    if (!rst_n || flush_ex) begin
        sigBus_br_ex <= 7'b0;
        sigBus_ls_ex <= 7'b0;
        regAddr_rd_br_ex <= 5'b0;
        regAddr_rd_ls_ex <= 5'b0;
        opcode_br_ex <= 10'b0;
        opcode_ls_ex <= 10'b0;
        func_br_ex  <= 7'b0;
        func_ls_ex  <= 7'b0;
        imm_br_ex   <= 32'b0;
        imm_ls_ex   <= 32'b0;
        regAddr_rj_br_ex <= 5'b0;
        regAddr_rk_br_ex <= 5'b0;
        regAddr_rj_ls_ex <= 5'b0;
        regAddr_rk_ls_ex <= 5'b0;
        regData_rj_br_ex <= 32'b0;
        regData_rk_br_ex <= 32'b0;
        regData_rj_ls_ex <= 32'b0;
        regData2_ls_ex   <= 32'b0;
        csrBus_br_ex     <= 17'b0;
        csrBus_ls_ex     <= 17'b0;
        evalid_br_ex     <= 1'b0;
        evalid_ls_ex     <= 1'b0;
        ecode_br_ex      <= 6'b0;
        ecode_ls_ex      <= 6'b0;
        pc_low_ex       <= 32'b0;
    end else if (!stall) begin
        // 控制 & 译码
        sigBus_br_ex <= sigBus_br_id;
        sigBus_ls_ex <= sigBus_ls_id;
        opcode_br_ex  <= opcode_br_id;
        opcode_ls_ex  <= opcode_ls_id;
        func_br_ex <= func_br_id;
        func_ls_ex <= func_ls_id;
        imm_br_ex  <= imm_br_id;
        imm_ls_ex  <= imm_ls_id;
        // 目标寄存器地址 (仅保留 rd → WB)
        regAddr_rd_br_ex <= regAddr_br_id[4:0];
        regAddr_rd_ls_ex <= regAddr_ls_id[4:0];
        // 源寄存器地址 (供前递比较)
        regAddr_rj_br_ex <= regAddr_br_id[9:5];
        regAddr_rk_br_ex <= regAddr_br_id[14:10];
        regAddr_rj_ls_ex <= regAddr_ls_id[9:5];
        regAddr_rk_ls_ex <= regAddr_ls_id[14:10];
        // 寄存器值
        regData_rj_br_ex <= regData_rj_br_id;
        regData_rk_br_ex <= regData_rk_br_id;
        regData_rj_ls_ex <= regData_rj_ls_id;
        regData2_ls_ex <= regData_rk_ls_id;
        csrBus_br_ex  <= csrBus_br_id;
        csrBus_ls_ex  <= csrBus_ls_id;
        evalid_br_ex  <= evalid_br_id;
        evalid_ls_ex  <= evalid_ls_id;
        ecode_br_ex   <= ecode_br_id;
        ecode_ls_ex   <= ecode_ls_id;
        // PC
        pc_low_ex   <= pc_low_id;
    end
end

// ============================================================
// EX阶段前递处理
// 优先级: ls_mem > br_mem > ls_wb > br_wb
// ============================================================

reg [31:0] i_finalExData_rj_br;
reg [31:0] i_finalExData_rk_br;
reg [31:0] i_finalExData_rj_ls;
reg [31:0] i_finalExData_rk_ls;
reg [31:0] i_finalExData_csr_br;
reg [31:0] i_finalExData_csr_ls;

always @(*) begin
    // ── 槽0 rj ──
    i_finalExData_rj_br = regData_rj_br_ex;
    if (regAddr_rj_br_ex == regAddr_rd_ls_mem && sigBus_ls_mem[6] && sigBus_ls_mem[1])
        i_finalExData_rj_br = aluResult_ls_mem;
    else if (regAddr_rj_br_ex == regAddr_rd_br_mem && sigBus_br_mem[6] && sigBus_br_mem[1])
        i_finalExData_rj_br = aluResult_br_mem;
    else if ((regAddr_rj_br_ex == regAddr_rd_ls_wb && regWrite_ls_wb)
          && (regAddr_rj_br_ex == regAddr_rd_br_wb && regWrite_br_wb))
        i_finalExData_rj_br = high_ls_wb ? regData_ls_wb : regData_br_wb;
    else if (regAddr_rj_br_ex == regAddr_rd_ls_wb && regWrite_ls_wb)
        i_finalExData_rj_br = regData_ls_wb;
    else if (regAddr_rj_br_ex == regAddr_rd_br_wb && regWrite_br_wb)
        i_finalExData_rj_br = regData_br_wb;

    // ── 槽0 rk ──
    i_finalExData_rk_br = regData_rk_br_ex;
    if (regAddr_rk_br_ex == regAddr_rd_ls_mem && sigBus_ls_mem[6] && sigBus_ls_mem[1])
        i_finalExData_rk_br = aluResult_ls_mem;
    else if (regAddr_rk_br_ex == regAddr_rd_br_mem && sigBus_br_mem[6] && sigBus_br_mem[1])
        i_finalExData_rk_br = aluResult_br_mem;
    else if ((regAddr_rk_br_ex == regAddr_rd_ls_wb && regWrite_ls_wb)
          && (regAddr_rk_br_ex == regAddr_rd_br_wb && regWrite_br_wb))
        i_finalExData_rk_br = high_ls_wb ? regData_ls_wb : regData_br_wb;
    else if (regAddr_rk_br_ex == regAddr_rd_ls_wb && regWrite_ls_wb)
        i_finalExData_rk_br = regData_ls_wb;
    else if (regAddr_rk_br_ex == regAddr_rd_br_wb && regWrite_br_wb)
        i_finalExData_rk_br = regData_br_wb;

    // ── 槽1 rj ──
    i_finalExData_rj_ls = regData_rj_ls_ex;
    if (regAddr_rj_ls_ex == regAddr_rd_ls_mem && sigBus_ls_mem[6] && sigBus_ls_mem[1])
        i_finalExData_rj_ls = aluResult_ls_mem;
    else if (regAddr_rj_ls_ex == regAddr_rd_br_mem && sigBus_br_mem[6] && sigBus_br_mem[1])
        i_finalExData_rj_ls = aluResult_br_mem;
    else if ((regAddr_rj_ls_ex == regAddr_rd_ls_wb && regWrite_ls_wb)
          && (regAddr_rj_ls_ex == regAddr_rd_br_wb && regWrite_br_wb))
        i_finalExData_rj_ls = high_ls_wb ? regData_ls_wb : regData_br_wb;
    else if (regAddr_rj_ls_ex == regAddr_rd_ls_wb && regWrite_ls_wb)
        i_finalExData_rj_ls = regData_ls_wb;
    else if (regAddr_rj_ls_ex == regAddr_rd_br_wb && regWrite_br_wb)
        i_finalExData_rj_ls = regData_br_wb;

    // ── 槽1 rk ──
    i_finalExData_rk_ls = regData2_ls_ex;
    if (regAddr_rk_ls_ex == regAddr_rd_ls_mem && sigBus_ls_mem[6] && sigBus_ls_mem[1])
        i_finalExData_rk_ls = aluResult_ls_mem;
    else if (regAddr_rk_ls_ex == regAddr_rd_br_mem && sigBus_br_mem[6] && sigBus_br_mem[1])
        i_finalExData_rk_ls = aluResult_br_mem;
    else if ((regAddr_rk_ls_ex == regAddr_rd_ls_wb && regWrite_ls_wb)
          && (regAddr_rk_ls_ex == regAddr_rd_br_wb && regWrite_br_wb))
        i_finalExData_rk_ls = high_ls_wb ? regData_ls_wb : regData_br_wb;
    else if (regAddr_rk_ls_ex == regAddr_rd_ls_wb && regWrite_ls_wb)
        i_finalExData_rk_ls = regData_ls_wb;
    else if (regAddr_rk_ls_ex == regAddr_rd_br_wb && regWrite_br_wb)
        i_finalExData_rk_ls = regData_br_wb;
end

// ============================================================
// CSR 前递: 流水线中有未完成的 CSR 写且地址匹配 → 前递
//   优先级: br_mem > br_wb > CSR 组合读
// ============================================================
always @(*) begin
    // ── 槽0 ──
    i_finalExData_csr_br = csr_rdata_br;
    if (|csrBus_br_mem[2:1] && csrBus_br_mem[16:3] == csrBus_br_ex[16:3])
        i_finalExData_csr_br = csrResult_br_mem;
    else if (|csrBus_br_wb[2:1] && csrBus_br_wb[16:3] == csrBus_br_ex[16:3])
        i_finalExData_csr_br = csrResult_br_wb;

    // ── 槽1 ──
    i_finalExData_csr_ls = csr_rdata_ls;
    if (|csrBus_br_ex[2:1] && csrBus_br_ex[16:3] == csrBus_ls_ex[16:3])
        i_finalExData_csr_ls = csrResult_br_ex;            // 同拍槽0 CSR 写 → 槽1 读
    else if (|csrBus_br_mem[2:1] && csrBus_br_mem[16:3] == csrBus_ls_ex[16:3])
        i_finalExData_csr_ls = csrResult_br_mem;
    else if (|csrBus_br_wb[2:1] && csrBus_br_wb[16:3] == csrBus_ls_ex[16:3])
        i_finalExData_csr_ls = csrResult_br_wb;
end


// ============================================================
// WB 级异常仲裁: 老指令优先 (high=1 为高位伴生, 取 high=0 的槽)
// ============================================================
wire except_br = evalid_br_wb && (evalid_ls_wb ?  high_ls_wb : 1'b1);  // LS高→BR老→取BR
wire except_ls = evalid_ls_wb && (evalid_br_wb ? !high_ls_wb : 1'b1);  // BR高→LS老→取LS



// 对应槽0: ALU / Branch (使用流水线后 EX 级信号)
EX_ALU uea(
    .clk(clk),
    .en(1'b1),
    .branch(sigBus_br_ex[4]),
    .jump(sigBus_br_ex[5]),
    .opcode(opcode_br_ex),
    .func(func_br_ex),
    .reg1(i_finalExData_rj_br),
    .reg2(i_finalExData_rk_br),
    .imm(imm_br_ex),
    .pc(sigBus_br_ex[0] ? (pc_low_ex + 32'd4) : pc_low_ex),
    .alu_result(aluResult_br_ex),
    .csr_result(csrResult_br_ex),
    .jump_taken(jumpTaken_br_ex),
    .branch_taken(branch_taken_br_ex),
    .jump_addr(jumpAddr_br_ex),
    .csr_read(i_finalExData_csr_br),
    .csrBus(csrBus_br_ex),
    .era   (era_val),
    .prmd  (prmd_val)
);

EX_LS uels(
    .clk        (clk),
    .en         (1'b1),
    .memRead    (sigBus_ls_ex[3] && !gate_ls),
    .memWrite   (sigBus_ls_ex[2] && !gate_ls),
    .opcode     (opcode_ls_ex),
    .func       (func_ls_ex),
    .reg1       (i_finalExData_rj_ls),            // rj (基址)
    .reg2       (i_finalExData_rk_ls),            // rk (ALU op2) / rd (store时, 已由ID级mux读出)
    .imm        (imm_ls_ex),
    .pc         (sigBus_ls_ex[0] ? (pc_low_ex + 32'd4) : pc_low_ex),
    .alu_result (aluResult_ls_ex),
    .mem_we     (memWrite_ls_ex),
    .mem_size   (memSize_ls_ex),
    .mem_wdata  (memWdata_ls_ex),
    .csr_read   (i_finalExData_csr_ls),
    .csrBus     (csrBus_ls_ex)
);

inst_controll uic(
    .clk            (clk),
    .rst_n          (rst_n),
    .jump_taken     (jumpTaken_br_ex),
    .except_taken   (evalid_br_wb || evalid_ls_wb),
    .stall          (stall),
    .pc_except      (eentry_val),
    .nopl           (nopl),
    .noph           (noph),
    .pc_jump        (jumpAddr_br_ex),
    .dual_inst_raw  (flush_id ? {NOP,NOP} : dual_inst_raw),
    .pc_next        (pc_next),
    .pc_low         (pc_low_id),
    .dual_inst      (dual_inst)
);

// load 符号/零扩展标志: opcode[3] (inst[25])
//  0 = 符号扩展 (LDB / LDH)
//  1 = 零扩展 (LDBU / LDHU)
wire loadSignExt_ls_ex = opcode_ls_ex[3];


always @(posedge clk) begin
    if (!rst_n) begin
        sigBus_br_mem     <= 7'b0;
        sigBus_ls_mem     <= 7'b0;
        regAddr_rd_br_mem <= 5'b0;
        regAddr_rd_ls_mem <= 5'b0;
        regAddr_rk_ls_mem <= 5'b0;
        aluResult_br_mem  <= 32'b0;
        aluResult_ls_mem  <= 32'b0;
        memWrite_ls_mem   <= 1'b0;
        memSize_ls_mem    <= 2'b0;
        memAddr_ls_mem    <= 32'b0;
        memWdata_ls_mem   <= 32'b0;
        signExt_ls_mem    <= 1'b0;
        csrBus_br_mem     <= 17'b0;
        csrResult_br_mem  <= 32'b0;
        csrBus_ls_mem     <= 17'b0;
        evalid_br_mem     <= 1'b0;
        evalid_ls_mem     <= 1'b0;
        ecode_br_mem      <= 6'b0;
        ecode_ls_mem      <= 6'b0;
        pc_low_br_mem     <= 32'b0;
        pc_low_ls_mem     <= 32'b0;
    end else if (!stall) begin
        // ── 槽0: EX → relay → WB ──
        if (flush_br_mem) begin
            sigBus_br_mem     <= 7'b0;
            regAddr_rd_br_mem <= 5'b0;
            aluResult_br_mem  <= 32'b0;
            csrBus_br_mem     <= 17'b0;
            csrResult_br_mem  <= 32'b0;
            evalid_br_mem     <= 1'b0;
            ecode_br_mem      <= 6'b0;
            pc_low_br_mem     <= 32'b0;
        end else begin
            regAddr_rd_br_mem  <= regAddr_rd_br_ex;
            aluResult_br_mem <= aluResult_br_ex;
            sigBus_br_mem     <= sigBus_br_ex;
            csrBus_br_mem     <= csrBus_br_ex;
            csrResult_br_mem  <= csrResult_br_ex;
            evalid_br_mem     <= evalid_br_ex;
            ecode_br_mem      <= ecode_br_ex;
            pc_low_br_mem     <= sigBus_br_ex[0] ? (pc_low_ex + 4) : pc_low_ex;
        end

        // ── 槽1: EX → MEM ──
        if (flush_ls_mem) begin
            sigBus_ls_mem     <= 7'b0;
            memWrite_ls_mem   <= 1'b0;
            memSize_ls_mem    <= 2'b0;
            memAddr_ls_mem    <= 32'b0;
            memWdata_ls_mem   <= 32'b0;
            regAddr_rd_ls_mem <= 5'b0;
            regAddr_rk_ls_mem <= 5'b0;
            aluResult_ls_mem  <= 32'b0;
            signExt_ls_mem    <= 1'b0;
            csrBus_ls_mem     <= 17'b0;
            evalid_ls_mem     <= 1'b0;
            ecode_ls_mem      <= 6'b0;
            pc_low_ls_mem     <= 32'b0;
        end else begin
            memWrite_ls_mem    <= memWrite_ls_ex;
            memSize_ls_mem  <= memSize_ls_ex;
            memAddr_ls_mem  <= aluResult_ls_ex;
            memWdata_ls_mem <= memWdata_ls_ex;
            // WB 穿越
            regAddr_rd_ls_mem       <= regAddr_rd_ls_ex;
            regAddr_rk_ls_mem       <= regAddr_rk_ls_ex;
            aluResult_ls_mem    <= aluResult_ls_ex;
            sigBus_ls_mem     <= sigBus_ls_ex;
            signExt_ls_mem   <= loadSignExt_ls_ex;
            csrBus_ls_mem    <= csrBus_ls_ex;
            evalid_ls_mem    <= evalid_ls_ex || ale;
            ecode_ls_mem     <= evalid_ls_ex ? ecode_ls_ex :
                                ale          ? 6'h09      : 6'b0;
            pc_low_ls_mem    <= sigBus_ls_ex[0] ? (pc_low_ex + 4) : pc_low_ex;
        end
    end
end

// ============================================================
// MEM → WB 流水线寄存器
//   dmem 同步读有 1 拍延迟, 控制信号再打一拍对齐, 直接驱动 WB 端口
// ============================================================
reg [31:0] aluResult_ls_wb;                 // ALU 结果 (非 load 时写回)
reg        memRead_ls_wb;                // load 标志 (mux 选择)
reg        loadSignExt_ls_wb;                // 符号扩展标志
wire [31:0] memRdata_ls_wb;              // dmem 读出数据
reg [1:0]  memSize_ls_wb;                // 访存宽度 (传递到 WB)
reg        signExt_ls_wb;                 // 符号扩展标志 (传递到 WB)
reg [1:0]  memAddr_low_wb;                // data_addr[1:0] (延迟1拍, 读对齐用)

always @(posedge clk) begin
    if (!rst_n) begin
        regWrite_br_wb    <= 1'b0;
        regWrite_ls_wb    <= 1'b0;
        high_br_wb        <= 1'b0;
        high_ls_wb        <= 1'b0;
        regAddr_rd_br_wb  <= 5'b0;
        regAddr_rd_ls_wb  <= 5'b0;
        regData_br_wb     <= 32'b0;
        aluResult_ls_wb   <= 32'b0;
        memRead_ls_wb     <= 1'b0;
        loadSignExt_ls_wb <= 1'b0;
        memSize_ls_wb     <= 2'b0;
        signExt_ls_wb     <= 1'b0;
        memAddr_low_wb    <= 2'b0;
        csrBus_br_wb      <= 17'b0;
        csrResult_br_wb   <= 32'b0;
        csrBus_ls_wb      <= 17'b0;
        evalid_br_wb      <= 1'b0;
        evalid_ls_wb      <= 1'b0;
        ecode_br_wb       <= 6'b0;
        ecode_ls_wb       <= 6'b0;
        pc_low_br_wb      <= 32'b0;
        pc_low_ls_wb      <= 32'b0;
    end else if (!stall) begin
        // ── 槽0: MEM → WB ──
        regAddr_rd_br_wb  <= regAddr_rd_br_mem;
        regData_br_wb     <= aluResult_br_mem;
        regWrite_br_wb    <= sigBus_br_mem[1];
        high_br_wb        <= sigBus_br_mem[0];
        csrBus_br_wb      <= csrBus_br_mem;
        csrResult_br_wb   <= csrResult_br_mem;
        evalid_br_wb      <= evalid_br_mem;
        ecode_br_wb       <= ecode_br_mem;
        pc_low_br_wb      <= pc_low_br_mem;

        // ── 槽1: MEM → WB ──
        regAddr_rd_ls_wb    <= regAddr_rd_ls_mem;
        regWrite_ls_wb       <= sigBus_ls_mem[1];
        high_ls_wb          <= sigBus_ls_mem[0];
        aluResult_ls_wb    <= aluResult_ls_mem;
        memRead_ls_wb   <= sigBus_ls_mem[3];
        loadSignExt_ls_wb   <= signExt_ls_mem;
        memSize_ls_wb       <= memSize_ls_mem;
        signExt_ls_wb       <= signExt_ls_mem;
        memAddr_low_wb      <= memAddr_ls_mem[1:0];
        csrBus_ls_wb        <= csrBus_ls_mem;
        evalid_ls_wb        <= evalid_ls_mem;
        ecode_ls_wb         <= ecode_ls_mem;
        pc_low_ls_wb        <= pc_low_ls_mem;
    end
end

// ============================================================
// MEM级 store 数据前递
//   默认: memWdata_ls_mem (EX级已前递)
//   同对上拍 br/load 双写同一rd → 用 high 位仲裁
//   单独命中: br > load > default
// ============================================================
wire [31:0] i_memWdata_final;
wire        fwd_mem_br   = sigBus_br_mem[6] && sigBus_br_mem[1]
                        && (regAddr_rk_ls_mem == regAddr_rd_br_mem);
wire        fwd_mem_load = sigBus_ls_mem[3] && (regAddr_rk_ls_mem == regAddr_rd_ls_mem);
wire        fwd_mem_both = fwd_mem_br && fwd_mem_load;

assign i_memWdata_final = fwd_mem_both ? (sigBus_ls_mem[0] ? memRdata_ls_wb : aluResult_br_mem) :
                          fwd_mem_br   ? aluResult_br_mem :
                          fwd_mem_load ? memRdata_ls_wb :
                                         memWdata_ls_mem;




MEM_Stage umem (
    .clk        (clk),
    .rst_n      (rst_n),
    .mem_req    (sigBus_ls_mem[2] || sigBus_ls_mem[3]),
    .wr_en      (memWrite_ls_mem),
    .mem_size   (memSize_ls_mem),
    .data_addr  (mem_pa),
    .write_data (i_memWdata_final),
    .write_strb (mem_write_strb),
    .signExt_wb (signExt_ls_wb),
    .mem_size_wb(memSize_ls_wb),
    .addr_low_wb(memAddr_low_wb),
    .read_data  (memRdata_ls_wb),
    .cpu_stall  (dcache_stall),
    .arvalid    (d_arvalid),
    .araddr     (d_araddr),
    .arlen      (d_arlen),
    .arsize     (d_arsize),
    .arready    (d_arready),
    .rvalid     (d_rvalid),
    .rdata      (d_rdata),
    .rresp      (d_rresp),
    .rlast      (d_rlast),
    .rready     (d_rready),
    .awvalid    (d_awvalid),
    .awaddr     (d_awaddr),
    .awlen      (d_awlen),
    .awsize     (d_awsize),
    .awready    (d_awready),
    .wvalid     (d_wvalid),
    .wdata      (d_wdata),
    .wstrb      (d_wstrb),
    .wlast      (d_wlast),
    .wready     (d_wready),
    .bvalid     (d_bvalid),
    .bresp      (d_bresp),
    .bid        (d_bid),
    .bready     (d_bready)
);

// ── MEM 级 byte write strobe ──
assign mem_write_strb =
    (memSize_ls_mem == 2'b00) ? (4'b0001 << memAddr_ls_mem[1:0]) :
    (memSize_ls_mem == 2'b01) ? (4'b0011 << {memAddr_ls_mem[1], 1'b0}) :
                                 4'b1111;

// ============================================================
// AXI 转接桥 (dCache 优先于 iCache)
// ============================================================
axi_bridge u_axi_bridge (
    .clk         (clk),
    .rst_n       (rst_n),

    .i_arvalid   (i_arvalid),
    .i_araddr    (i_araddr),
    .i_arlen     (i_arlen),
    .i_arsize    (i_arsize),
    .i_arready   (i_arready),
    .i_rvalid    (i_rvalid),
    .i_rdata     (i_rdata),
    .i_rresp     (i_rresp),
    .i_rlast     (i_rlast),
    .i_rready    (i_rready),

    .d_arvalid   (d_arvalid),
    .d_araddr    (d_araddr),
    .d_arlen     (d_arlen),
    .d_arsize    (d_arsize),
    .d_arready   (d_arready),
    .d_rvalid    (d_rvalid),
    .d_rdata     (d_rdata),
    .d_rresp     (d_rresp),
    .d_rlast     (d_rlast),
    .d_rready    (d_rready),

    .d_awvalid   (d_awvalid),
    .d_awaddr    (d_awaddr),
    .d_awlen     (d_awlen),
    .d_awsize    (d_awsize),
    .d_awready   (d_awready),
    .d_wvalid    (d_wvalid),
    .d_wdata     (d_wdata),
    .d_wstrb     (d_wstrb),
    .d_wlast     (d_wlast),
    .d_wready    (d_wready),
    .d_bvalid    (d_bvalid),
    .d_bresp     (d_bresp),
    .d_bid       (d_bid),
    .d_bready    (d_bready),

    .arid    (arid),
    .araddr  (araddr),
    .arlen   (arlen),
    .arsize  (arsize),
    .arburst (arburst),
    .arlock  (arlock),
    .arcache (arcache),
    .arprot  (arprot),
    .arvalid (arvalid),
    .arready (arready),
    .rid     (rid),
    .rdata   (rdata),
    .rresp   (rresp),
    .rlast   (rlast),
    .rvalid  (rvalid),
    .rready  (rready),
    .awid    (awid),
    .awaddr  (awaddr),
    .awlen   (awlen),
    .awsize  (awsize),
    .awburst (awburst),
    .awlock  (awlock),
    .awcache (awcache),
    .awprot  (awprot),
    .awvalid (awvalid),
    .awready (awready),
    .wid     (wid),
    .wdata   (wdata),
    .wstrb   (wstrb),
    .wlast   (wlast),
    .wvalid  (wvalid),
    .wready  (wready),
    .bid     (bid),
    .bresp   (bresp),
    .bvalid  (bvalid),
    .bready  (bready)
);

// ============================================================
// WB 级写回 (regData_ls_wb 为 mux 输出, 其余端口由寄存器直接驱动)
// ============================================================
assign regData_ls_wb = memRead_ls_wb ? memRdata_ls_wb : aluResult_ls_wb;

Regs urg(
    .clk(clk),
    .en(1'b1),
    // 写端口 (WB → Regs)
    .regWrite0(regWrite_br_wb && !flush_br_wb),
    .regWrite1(regWrite_ls_wb && !flush_ls_wb),
    .write_addr0(regAddr_rd_br_wb),
    .write_addr1(regAddr_rd_ls_wb),
    .write_data0(regData_br_wb),
    .write_data1(regData_ls_wb),
    // 读端口 (ID → Regs, 地址直连)
    .read_addr01(regAddr_br_id[9:5]),   
    .read_addr02(regAddr_br_id[14:10]), 
    .read_addr11(regAddr_ls_id[9:5]),   
    .read_addr12(regAddr_ls_id[14:10]), 
    // 读出数据
    .read01(regData_rj_br_id),
    .read02(regData_rk_br_id),
    .read11(regData_rj_ls_id),
    .read12(regData_rk_ls_id)
);


csr ucsr(
    .clk(clk), 
    .rst_n(rst_n),
    .wea(|csrBus_br_wb[2:1] && !flush_br_wb),
    .except     (except_br || except_ls),
    .except_ecode(except_br ? ecode_br_wb : ecode_ls_wb),
    .except_era (except_br ? pc_low_br_wb : pc_low_ls_wb),
    .except_badv(
        (except_br && ecode_br_wb == 6'h08) ? pc_low_br_wb  :
        (except_ls && ecode_ls_wb == 6'h08) ? pc_low_ls_wb  :
        (except_ls && ecode_ls_wb == 6'h09) ? aluResult_ls_wb : 32'b0
    ),
    .ertn       (csrBus_br_wb[1] && (csrBus_br_wb[16:3] == 14'h0)),  // ERTN 写 CRMD    .hwi        (hwi),
    .ipi        (ipi),    .raddr0(csrBus_br_ex[16:3]),
    .raddr1(csrBus_ls_ex[16:3]),
    .waddr(csrBus_br_wb[16:3]),
    .wdata(csrResult_br_wb),       // CSR 写入数据 (EX 级 csrResult 流水至 WB)
    .plv0(plv0),
    .rdata0(csr_rdata_br),         // → EX 级 csr_read
    .rdata1(csr_rdata_ls),         // → EX 级 csr_read
    .eentry_val(eentry_val),
    .era_val   (era_val),
    .prmd_val  (prmd_val),
    .int_pending(int_pending),
    .da         (da),
    .pg         (pg),
    .plv        (plv),
    .dmw0_val   (dmw0_val),
    .dmw1_val   (dmw1_val)
);

// ============================================================
// 流水线控制 (分支冲刷)
// ============================================================
pipeline_controll upipe_ctrl (
    .branch_taken (jumpTaken_br_ex),
    .high_ls_mem  (sigBus_ls_ex[0]),
    .high_ls_wb   (high_ls_wb),
    .evalid_br_wb (evalid_br_wb),
    .evalid_ls_wb (evalid_ls_wb),
    .stall_dcache (dcache_stall),
    .stall_icache (if_stall),
    .flush_id     (flush_id),
    .flush_ex     (flush_ex),
    .flush_br_mem (flush_br_mem),
    .flush_ls_mem (flush_ls_mem),
    .flush_br_wb  (flush_br_wb),
    .flush_ls_wb  (flush_ls_wb),
    .stall_out    (stall)
);



endmodule