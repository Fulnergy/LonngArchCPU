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
wire [31:0] pc_next;                    // inst_controll 输出的下一 PC
wire [63:0] dual_inst_raw;              // IF 取出的原始指令对
wire [63:0] dual_inst;                  // inst_controll 调整后的指令对
wire        if_en;                      // IF 级使能 (暂不 stall)


// ID Stage 输出
wire [9:0]  opc0_id, opc1_id;
wire [6:0]  func0_id, func1_id;
wire [4:0]  sigs0_id, sigs1_id;
wire [31:0] imm0, imm1;
wire [14:0] regs0_id, regs1_id;
wire        nop0, nop1;

// ── Regs 写端口 (WB 级驱动) ──
reg  [4:0]  wb_waddr0, wb_waddr1;       // 写回地址 (rd)
reg  [31:0] wb_wdata0;                   // 写回数据 槽0
wire [31:0] wb_wdata1;                   // 写回数据 槽1 (mux 输出, 保持 wire)
reg         wb_rw0, wb_rw1;             // 写使能

// ── Regs 读端口数据 ──
wire [31:0] rdata01, rdata02;           // 槽0 读出 (rj, rk)
wire [31:0] rdata11, rdata12;           // 槽1 读出 (rj, rk)

// 槽1 read_addr12 mux: store 时读 rd, 否则读 rk
wire [4:0] slot1_addr12;

// ── EX 级信号 ──
wire [31:0] alu_result0;              // 槽0 ALU 结果
wire        jump_taken0;
wire [31:0] jump_addr0;

// ── 槽1: EX_LS (ALU + Load/Store 地址计算) ──
wire [31:0] alu_result_ls;            // 访存地址 / ALU 结果
wire        mem_we_ls;                // 存储器写使能
wire [1:0]  mem_size_ls;              // 访存宽度: 00=byte, 01=half, 10=word
wire [31:0] mem_wdata_ls;             // 写入存储器的数据

wire [31:0] mem_rdata;                // load 读回数据

assign if_en = 1'b1;

IF_Stage #(
    .IMEM_FILE (IMEM_FILE)
) uif (
    .clk        (clk),
    .en         (if_en),
    .pc         (pc_next[12:0]),
    .dual_inst  (dual_inst_raw)
);

inst_controll uic(
    .clk            (clk),
    .rst_n          (rst_n),
    .jump_taken     (jump_taken0),
    .nop0           (nop0),
    .nop1           (nop1),
    .pc_jump        (jump_addr0),
    .pc_last        (pc_next),
    .dual_inst_raw  (dual_inst_raw),
    .pc_next        (pc_next),
    .dual_inst      (dual_inst)
);

reg [31:0] pc_id;
//使pc跟随流水线流动，以使pcaddu12等得到正确结果
always @(posedge clk) begin
    pc_id<=pc_next;
end

ID_Stage uid(
    .dual_inst(dual_inst),
    .opc0(opc0_id),
    .opc1(opc1_id),
    .func0(func0_id),
    .func1(func1_id),
    .sigs0(sigs0_id),
    .sigs1(sigs1_id),
    .imm0(imm0),
    .imm1(imm1),
    .regs0(regs0_id),
    .regs1(regs1_id),
    .nop0(nop0),
    .nop1(nop1)
);


assign slot1_addr12 = sigs1_id[1] ? regs1_id[4:0] : regs1_id[14:10];
                                           // memWrite=1 → rd, else → rk

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
    .read_addr12(slot1_addr12),         // 槽1 rk / rd (store时切换)
    // 读出数据
    .read01(rdata01),
    .read02(rdata02),
    .read11(rdata11),
    .read12(rdata12)
);


// ============================================================
// ID -> EX 流水线寄存器
// ============================================================
reg [9:0]  opc0_ex, opc1_ex;
reg [6:0]  func0_ex, func1_ex;
reg [4:0]  sigs0_ex, sigs1_ex;
reg [4:0]  rd0_ex, rd1_ex;            // 目标寄存器地址 (仅保留 rd, 供 WB 写回)
reg [31:0] imm0_ex, imm1_ex;
reg [31:0] pc_ex;
reg [31:0] rd01_ex, rd02_ex;          // 槽0 寄存器值 (流水线后)
reg [31:0] rd11_ex, rd12_ex;          // 槽1 寄存器值 (流水线后: rj, rk/rd*)

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
    // 目标寄存器地址 (仅保留 rd → WB)
    rd0_ex <= regs0_id[4:0];
    rd1_ex <= regs1_id[4:0];
    // 寄存器值
    rd01_ex <= rdata01;
    rd02_ex <= rdata02;
    rd11_ex <= rdata11;
    rd12_ex <= rdata12;
    // PC
    pc_ex   <= pc_id;
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

EX_LS uels(
    .clk        (clk),
    .en         (1'b1),
    .memRead    (sigs1_ex[2]),        // sigs = {jump, branch, memRead, memWrite, regWrite}
    .memWrite   (sigs1_ex[1]),
    .opcode     (opc1_ex),
    .func       (func1_ex),
    .reg1       (rd11_ex),            // rj (基址)
    .reg2       (rd12_ex),            // rk (ALU op2) / rd (store时, 已由ID级mux读出)
    .store_data (rd12_ex),            // store: rd12_ex 即为 rd 的值
    .imm        (imm1_ex),
    .alu_result (alu_result_ls),
    .mem_we     (mem_we_ls),
    .mem_size   (mem_size_ls),
    .mem_wdata  (mem_wdata_ls)
);

// load 符号/零扩展标志: opcode[3] (inst[25])
//  0 = 符号扩展 (LDB / LDH)
//  1 = 零扩展 (LDBU / LDHU)
wire signExt_ls = opc1_ex[3];

// ============================================================
// EX → MEM 流水线寄存器
//   slot 1: 访存控制 + WB 穿越信号
//   slot 0: 中继气泡 (与 slot 1 同步到达 WB)
// ============================================================

// ── 槽0 中继: EX → relay → WB (NBA 逐级传递, 2 拍对齐槽1) ──
reg [4:0]  rd0_relay;
reg [31:0] alu0_relay;
reg        rw0_relay;

// ── 槽1 EX→MEM (访存 + WB 穿越) ──
reg        mem_we_mem;
reg [1:0]  mem_size_mem;
reg [31:0] mem_addr_mem;
reg [31:0] mem_wdata_mem;
// WB 穿越信号
reg [4:0]  rd1_mem;
reg [31:0] alu_ls_mem;
reg        rw1_mem;
reg        memRead_mem;
reg        signExt_mem;               // load 符号扩展标志

always @(posedge clk) begin
    // ── 槽0: EX → relay → WB (NBA 保证 relay→wb_* 差 1 拍) ──
    rd0_relay  <= rd0_ex;
    alu0_relay <= alu_result0;
    rw0_relay  <= sigs0_ex[0];
    wb_waddr0  <= rd0_relay;           // 直接写 WB 端口
    wb_wdata0  <= alu0_relay;
    wb_rw0     <= rw0_relay;

    // ── 槽1: EX → MEM ──
    mem_we_mem    <= mem_we_ls;
    mem_size_mem  <= mem_size_ls;
    mem_addr_mem  <= alu_result_ls;
    mem_wdata_mem <= mem_wdata_ls;
    // WB 穿越
    rd1_mem       <= rd1_ex;
    alu_ls_mem    <= alu_result_ls;
    rw1_mem       <= sigs1_ex[0];
    memRead_mem   <= sigs1_ex[2];
    signExt_mem   <= signExt_ls;
end

// ============================================================
// MEM → WB 流水线寄存器
//   dmem 同步读有 1 拍延迟, 控制信号再打一拍对齐, 直接驱动 WB 端口
// ============================================================
reg [31:0] alu_ls_wb;                 // ALU 结果 (非 load 时写回)
reg        memRead_wb;                // load 标志 (mux 选择)
reg        signExt_wb;                // 符号扩展标志
reg [31:0] mem_rdata_wb;              // dmem 读出数据

always @(posedge clk) begin
    wb_waddr1    <= rd1_mem;          // 直接写 WB 端口
    wb_rw1       <= rw1_mem;
    alu_ls_wb    <= alu_ls_mem;
    memRead_wb   <= memRead_mem;
    signExt_wb   <= signExt_mem;
    mem_rdata_wb <= mem_rdata;
end

// ── MEM Stage: 数据存储器 ──

MEM_Stage #(
    .ADDR_WIDTH (15),
    .DATA_WIDTH (32),
    .INIT_FILE  (DMEM_FILE)
) umem (
    .clk        (clk),
    .wr_en      (mem_we_mem),
    .signExt    (signExt_mem),
    .mem_size   (mem_size_mem),
    .data_addr  (mem_addr_mem[16:0]),    // 字节地址 (17b → 128KB)
    .write_data (mem_wdata_mem),
    .read_data  (mem_rdata)
);

// ============================================================
// WB 级写回 (wb_wdata1 为 mux 输出, 其余端口由寄存器直接驱动)
// ============================================================
assign wb_wdata1 = memRead_wb ? mem_rdata_wb : alu_ls_wb;

endmodule