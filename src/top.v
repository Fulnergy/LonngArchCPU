//双发射，槽0可branch,槽1可load/store，二者都兼容alu
module top #(
    parameter IMEM_FILE  = "imem_init.hex" ,
    parameter DMEM_FILE  = "dmem_init.hex"
)
(
    input clk,
    input rst_n
);

// ── PC & IF 信号 ──
wire [31:0] pc_if;                    // inst_controll 输出的下一 PC
wire [63:0] dual_inst_raw;              // IF 取出的原始指令对
wire [63:0] dual_inst;                  // inst_controll 调整后的指令对
wire        if_en;                      // IF 级使能 (暂不 stall)


// ID Stage 输出
wire [9:0]  opcode_br_id, opcode_ls_id;
wire [6:0]  func_br_id, func_ls_id;
wire [4:0]  sigBus_br_id, sigBus_ls_id;
wire [31:0] imm_br_id, imm_ls_id;
wire [14:0] regAddr_br_id, regAddr_ls_id;
wire        nop0, nop1;

// ── Regs 写端口 (WB 级驱动) ──
reg  [4:0]  regAddr_rd_br_wb, regAddr_rd_ls_wb;       // 写回地址 (rd)
reg  [31:0] regData_br_wb;                   // 写回数据 槽0
wire [31:0] regData_ls_wb;                   // 写回数据 槽1 (mux 输出, 保持 wire)
reg         regWrite_br_wb, regWrite_ls_wb;             // 写使能

// ── Regs 读端口数据 ──
wire [31:0] regData_rj_br_id, regData_rk_br_id;           // 槽0 读出 (rj, rk)
wire [31:0] regData_rj_ls_id, regData2_ls_id;           // 槽1 读出 (rj, rk)

// 槽1 read_addr12 mux: store 时读 rd, 否则读 rk
wire [4:0] i_regAddr_rt_ls_id;

// ── EX 级信号 ──
wire [31:0] aluResult_br_ex;              // 槽0 ALU 结果
wire        jumpTaken_br_ex;
wire [31:0] jumpAddr_br_ex;

// ── 槽1: EX_LS (ALU + Load/Store 地址计算) ──
wire [31:0] aluResult_ls_ex;            // 访存地址 / ALU 结果
wire        memWrite_ls_ex;                // 存储器写使能
wire [1:0]  memSize_ls_ex;              // 访存宽度: 00=byte, 01=half, 10=word
wire [31:0] memWdata_ls_ex;             // 写入存储器的数据

wire [31:0] memRdata_ls_mem;                // load 读回数据

assign if_en = 1'b1;

IF_Stage #(
    .IMEM_FILE (IMEM_FILE)
) uif (
    .clk        (clk),
    .en         (if_en),
    .pc         (pc_if[12:0]),
    .dual_inst  (dual_inst_raw)
);

inst_controll uic(
    .clk            (clk),
    .rst_n          (rst_n),
    .jump_taken     (jumpTaken_br_ex),
    .nop0           (nop0),
    .nop1           (nop1),
    .pc_jump        (jumpAddr_br_ex),
    .pc_last        (pc_if),
    .dual_inst_raw  (dual_inst_raw),
    .pc_next        (pc_if),
    .dual_inst      (dual_inst)
);

reg [31:0] pc_id;
//使pc跟随流水线流动，以使pcaddu12等得到正确结果
always @(posedge clk) begin
    pc_id<=pc_if;
end

ID_Stage uid(
    .dual_inst(dual_inst),
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
    .nop0(nop0),
    .nop1(nop1)
);


assign i_regAddr_rt_ls_id = sigBus_ls_id[1] ? regAddr_ls_id[4:0] : regAddr_ls_id[14:10];
                                           // memWrite=1 → rd, else → rk

Regs urg(
    .clk(clk),
    .en(1'b1),
    // 写端口 (WB → Regs)
    .regWrite0(regWrite_br_wb),
    .regWrite1(regWrite_ls_wb),
    .write_addr0(regAddr_rd_br_wb),
    .write_addr1(regAddr_rd_ls_wb),
    .write_data0(regData_br_wb),
    .write_data1(regData_ls_wb),
    // 读端口 (ID → Regs, 地址直连)
    .read_addr01(regAddr_br_id[9:5]),        // 槽0 rs = rj
    .read_addr02(regAddr_br_id[14:10]),      // 槽0 rt = rk
    .read_addr11(regAddr_ls_id[9:5]),        // 槽1 rs = rj
    .read_addr12(i_regAddr_rt_ls_id),         // 槽1 rk / rd (store时切换)
    // 读出数据
    .read01(regData_rj_br_id),
    .read02(regData_rk_br_id),
    .read11(regData_rj_ls_id),
    .read12(regData2_ls_id)
);


// ============================================================
// ID -> EX 流水线寄存器
// ============================================================
reg [9:0]  opcode_br_ex, opcode_ls_ex;
reg [6:0]  func_br_ex, func_ls_ex;
reg [4:0]  sigBus_br_ex, sigBus_ls_ex;
reg [4:0]  regAddr_rd_br_ex, regAddr_rd_ls_ex;            // 目标寄存器地址 (仅保留 rd, 供 WB 写回)
reg [31:0] imm_br_ex, imm_ls_ex;
reg [31:0] pc_ex;
reg [31:0] regData_rj_br_ex, regData_rk_br_ex;          // 槽0 寄存器值 (流水线后)
reg [31:0] regData_rj_ls_ex, regData2_ls_ex;          // 槽1 寄存器值 (流水线后: rj, rk/rd*)

always @(posedge clk) begin
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
    // 寄存器值
    regData_rj_br_ex <= regData_rj_br_id;
    regData_rk_br_ex <= regData_rk_br_id;
    regData_rj_ls_ex <= regData_rj_ls_id;
    regData2_ls_ex <= regData2_ls_id;
    // PC
    pc_ex   <= pc_id;
end

// 对应槽0: ALU / Branch (使用流水线后 EX 级信号)
EX_ALU uea(
    .clk(clk),
    .en(1'b1),
    .branch(sigBus_br_ex[3]),
    .jump(sigBus_br_ex[4]),
    .opcode(opcode_br_ex),
    .func(func_br_ex),
    .reg1(regData_rj_br_ex),
    .reg2(regData_rk_br_ex),
    .imm(imm_br_ex),
    .pc(pc_ex),
    .alu_result(aluResult_br_ex),
    .jump_taken(jumpTaken_br_ex),
    .jump_addr(jumpAddr_br_ex)
);

EX_LS uels(
    .clk        (clk),
    .en         (1'b1),
    .memRead    (sigBus_ls_ex[2]),        // sigs = {jump, branch, memRead, memWrite, regWrite}
    .memWrite   (sigBus_ls_ex[1]),
    .opcode     (opcode_ls_ex),
    .func       (func_ls_ex),
    .reg1       (regData_rj_ls_ex),            // rj (基址)
    .reg2       (regData2_ls_ex),            // rk (ALU op2) / rd (store时, 已由ID级mux读出)
    .store_data (regData2_ls_ex),            // store: regData2_ls_ex 即为 rd 的值
    .imm        (imm_ls_ex),
    .alu_result (aluResult_ls_ex),
    .mem_we     (memWrite_ls_ex),
    .mem_size   (memSize_ls_ex),
    .mem_wdata  (memWdata_ls_ex)
);

// load 符号/零扩展标志: opcode[3] (inst[25])
//  0 = 符号扩展 (LDB / LDH)
//  1 = 零扩展 (LDBU / LDHU)
wire loadSignExt_ls_ex = opcode_ls_ex[3];

// ============================================================
// EX → MEM 流水线寄存器
//   slot 1: 访存控制 + WB 穿越信号
//   slot 0: 中继气泡 (与 slot 1 同步到达 WB)
// ============================================================

// ── 槽0 中继: EX → relay → WB (NBA 逐级传递, 2 拍对齐槽1) ──
reg [4:0]  regAddr_rd_br_mem;
reg [31:0] aluResult_br_mem;
reg        regWrite_br_mem;

// ── 槽1 EX→MEM (访存 + WB 穿越) ──
reg        memWrite_ls_mem;
reg [1:0]  memSize_ls_mem;
reg [31:0] memAddr_ls_mem;
reg [31:0] memWdata_ls_mem;
// WB 穿越信号
reg [4:0]  regAddr_rd_ls_mem;
reg [31:0] aluResult_ls_mem;
reg        regWrite_ls_mem;
reg        memRead_ls_mem;
reg        signExt_ls_mem;               // load 符号扩展标志

always @(posedge clk) begin
    // ── 槽0: EX → relay → WB (NBA 保证 relay→wb_* 差 1 拍) ──
    regAddr_rd_br_mem  <= regAddr_rd_br_ex;
    aluResult_br_mem <= aluResult_br_ex;
    regWrite_br_mem  <= sigBus_br_ex[0];
    regAddr_rd_br_wb  <= regAddr_rd_br_mem;           // 直接写 WB 端口
    regData_br_wb  <= aluResult_br_mem;
    regWrite_br_wb     <= regWrite_br_mem;

    // ── 槽1: EX → MEM ──
    memWrite_ls_mem    <= memWrite_ls_ex;
    memSize_ls_mem  <= memSize_ls_ex;
    memAddr_ls_mem  <= aluResult_ls_ex;
    memWdata_ls_mem <= memWdata_ls_ex;
    // WB 穿越
    regAddr_rd_ls_mem       <= regAddr_rd_ls_ex;
    aluResult_ls_mem    <= aluResult_ls_ex;
    regWrite_ls_mem       <= sigBus_ls_ex[0];
    memRead_ls_mem   <= sigBus_ls_ex[2];
    signExt_ls_mem   <= loadSignExt_ls_ex;
end

// ============================================================
// MEM → WB 流水线寄存器
//   dmem 同步读有 1 拍延迟, 控制信号再打一拍对齐, 直接驱动 WB 端口
// ============================================================
reg [31:0] aluResult_ls_wb;                 // ALU 结果 (非 load 时写回)
reg        memRead_ls_wb;                // load 标志 (mux 选择)
reg        loadSignExt_ls_wb;                // 符号扩展标志
reg [31:0] memRdata_ls_wb;              // dmem 读出数据

always @(posedge clk) begin
    regAddr_rd_ls_wb    <= regAddr_rd_ls_mem;          // 直接写 WB 端口
    regWrite_ls_wb       <= regWrite_ls_mem;
    aluResult_ls_wb    <= aluResult_ls_mem;
    memRead_ls_wb   <= memRead_ls_mem;
    loadSignExt_ls_wb   <= signExt_ls_mem;
    memRdata_ls_wb <= memRdata_ls_mem;
end

// ── MEM Stage: 数据存储器 ──

MEM_Stage #(
    .ADDR_WIDTH (15),
    .DATA_WIDTH (32),
    .INIT_FILE  (DMEM_FILE)
) umem (
    .clk        (clk),
    .wr_en      (memWrite_ls_mem),
    .signExt    (signExt_ls_mem),
    .mem_size   (memSize_ls_mem),
    .data_addr  (memAddr_ls_mem[16:0]),    // 字节地址 (17b → 128KB)
    .write_data (memWdata_ls_mem),
    .read_data  (memRdata_ls_mem)
);

// ============================================================
// WB 级写回 (regData_ls_wb 为 mux 输出, 其余端口由寄存器直接驱动)
// ============================================================
assign regData_ls_wb = memRead_ls_wb ? memRdata_ls_wb : aluResult_ls_wb;

endmodule