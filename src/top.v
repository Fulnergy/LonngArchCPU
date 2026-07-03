//双发射，槽0可branch,槽1可load/store，二者都兼容alu
module top(
    input clk,
    input rst_n
);

// ── PC & IF 信号 ──
reg  [12:0] pc;                         // 程序计数器 (字节地址, 8 字节对齐)
wire [31:0] dual_inst;                  // 双发射指令对 {inst_ls, inst_alu}
wire        if_en;                      // IF 级使能 (stall 时拉低)

assign if_en = 1'b1;                    // TODO: 后续由 stall 控制逻辑驱动

// PC 更新: 默认 +8 (双发射), 分支跳转时载入目标地址
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pc <= 13'b0;
    else if (jump_taken0)
        pc <= jump_addr0[12:0];
    else if (if_en)
        pc <= pc + 13'd8;
end

IF_Stage uif(
    .clk        (clk),
    .en         (if_en),
    .inst_addr  (pc),
    .dual_inst  (dual_inst)
);

// ID Stage 输出
wire [9:0]  opc0_id, opc1_id;
wire [6:0]  func0_id, func1_id;
wire [4:0]  sigs0_id, sigs1_id;
wire [31:0] imm0, imm1;
wire [14:0] regs0_id, regs1_id;

ID_stage uid(
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
    .regs1(regs1_id)
);


// ── Regs 写端口 (待 WB 级驱动) ──
wire [4:0]  wb_waddr0, wb_waddr1;       // 写回地址 (rd)
wire [31:0] wb_wdata0, wb_wdata1;       // 写回数据
wire        wb_rw0, wb_rw1;             // 写使能

// ── Regs 读端口数据 ──
wire [31:0] rdata01, rdata02;           // 槽0 读出 (rj, rk)
wire [31:0] rdata11, rdata12;           // 槽1 读出 (rj, rk)

// 槽1 read_addr12 mux: store 时读 rd, 否则读 rk
wire [4:0] slot1_addr12;
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

// ── EX 级信号 ──
wire [31:0] alu_result0, alu_result1;
wire        jump_taken0;
wire [31:0] jump_addr0;

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

// ── 槽1: EX_LS (ALU + Load/Store 地址计算) ──
wire [31:0] alu_result_ls;            // 访存地址 / ALU 结果
wire        mem_we_ls;                // 存储器写使能
wire [1:0]  mem_size_ls;              // 访存宽度: 00=byte, 01=half, 10=word
wire [31:0] mem_wdata_ls;             // 写入存储器的数据

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

// ============================================================
// EX → MEM 流水线寄存器 (仅保留 MEM 阶段需要的信号)
// ============================================================
reg        mem_we_mem;
reg [1:0]  mem_size_mem;
reg [31:0] mem_addr_mem;
reg [31:0] mem_wdata_mem;

always @(posedge clk) begin
    mem_we_mem    <= mem_we_ls;
    mem_size_mem  <= mem_size_ls;
    mem_addr_mem  <= alu_result_ls;
    mem_wdata_mem <= mem_wdata_ls;
end

// ── byte_we 译码: mem_size + 地址低 2 位 → 字节写使能 ──
wire [3:0] byte_we_mem;
assign byte_we_mem =
    (mem_size_mem == 2'b00) ? (4'b0001 << mem_addr_mem[1:0]) :        // byte
    (mem_size_mem == 2'b01) ? (4'b0011 << {mem_addr_mem[1], 1'b0}) :  // halfword
                              4'b1111;                                  // word

// ── MEM Stage: 数据存储器 ──
wire [31:0] mem_rdata;                // load 读回数据

MEM_Stage #(
    .ADDR_WIDTH (15),
    .DATA_WIDTH (32),
    .INIT_FILE  ("dmem_init.hex")
) umem (
    .clk        (clk),
    .wr_en      (mem_we_mem),
    .byte_we    (byte_we_mem),
    .data_addr  (mem_addr_mem[14:0]),
    .write_data (mem_wdata_mem),
    .read_data  (mem_rdata)
);

endmodule