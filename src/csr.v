module csr (
    input          clk, rst_n,
    input          wea,
    input          except,                       // 异常触发 (优先级高于 wea)
    input  [5:0]   except_ecode,                 // 异常号
    input  [31:0]  except_era,                   // 异常指令 PC
    input  [31:0]  except_badv,                  // 出错虚地址
    input          ertn,                         // ERTN 触发
    input  [7:0]   hwi,                          // 硬件中断
    input          ipi,                          // 核间中断
    input  [13:0]  raddr0, raddr1, waddr,        // CSR 编号
    input  [31:0]  wdata,                        // 写入数据
    output         plv0,                          // 当前特权等级
    output [31:0]  rdata0, rdata1,               // 读出数据
    output [31:0]  eentry_val,                   // 异常入口地址
    output [31:0]  era_val,                      // ERA (供 ERTN)
    output [31:0]  prmd_val,                     // PRMD (供 ERTN)
    output         int_pending                   // 中断待处理
);

localparam
    CSR_CRMD   = 14'h0,
    CSR_PRMD   = 14'h1,
    CSR_ECFG   = 14'h4,
    CSR_ESTAT  = 14'h5,
    CSR_ERA    = 14'h6,
    CSR_BADV   = 14'h7,
    CSR_EENTRY = 14'hC,
    CSR_LLBCTL = 14'h60,
    CSR_TID    = 14'h40,
    CSR_TCFG   = 14'h41,
    CSR_TVAL   = 14'h42,
    CSR_TICLR  = 14'h44,
    CSR_SAVE0  = 14'h30,
    CSR_SAVE1  = 14'h31,
    CSR_SAVE2  = 14'h32,
    CSR_SAVE3  = 14'h33;

// 定时器位宽: n 由实现决定 (手册 7.6.2)
localparam TIMER_WIDTH = 28;

// 各寄存器定义
reg [31:0] crmd, prmd, ecfg, estat, era, badv, eentry, llbctl;
reg [31:0] tid, tcfg;
reg [31:0] save [0:3];

// ============================================================
// 读端口: 组合读, 双端口独立寻址
// ============================================================
function [31:0] csr_read;
    input [13:0] a;
    begin
        csr_read = (a == CSR_CRMD)   ? crmd :
                   (a == CSR_PRMD)   ? prmd :
                   (a == CSR_ECFG)   ? ecfg :
                   (a == CSR_ESTAT)  ? estat :
                   (a == CSR_ERA)    ? era :
                   (a == CSR_BADV)   ? badv :
                   (a == CSR_EENTRY) ? eentry :
                   (a == CSR_LLBCTL) ? llbctl :
                   (a == CSR_TID)    ? tid    :
                   (a == CSR_TCFG)   ? tcfg   :
                   (a == CSR_TVAL)   ? { {32-TIMER_WIDTH{1'b0}}, tval_comb } :
                   (a == CSR_TICLR)  ? 32'b0  :  // TICLR 读恒为 0
                   (a == CSR_SAVE0)  ? save[0] :
                   (a == CSR_SAVE1)  ? save[1] :
                   (a == CSR_SAVE2)  ? save[2] :
                   (a == CSR_SAVE3)  ? save[3] :
                                       32'b0;
    end
endfunction

assign rdata0 = csr_read(raddr0);
assign rdata1 = csr_read(raddr1);

// ============================================================
// plv0: CRMD.PLV==0 时为 1
// ============================================================
assign plv0 = (crmd[1:0] == 2'b0);
assign eentry_val = eentry;
assign era_val    = era;
assign prmd_val   = prmd;

// 中断检测: CRMD.IE=1 且任一局部使能中断挂起
wire [12:0] int_vec;
assign int_vec = {estat[12], estat[11], estat[9:2], estat[1:0]}
               & {ecfg[12:11], ecfg[9:0]};
assign int_pending = crmd[2] && (|int_vec);

// ============================================================
// CRMD (0x0): 当前模式信息
//   [1:0]  PLV   (RW, 合法值 0/3)
//   [2]    IE    (RW)
//   [3]    DA    (RW)
//   [4]    PG    (RW)
//   [6:5]  DATF  (RW)
//   [8:7]  DATM  (RW)
//   [31:9] R0
//   复位: DA=1, 其余 0
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        crmd <= 32'h00000008;      // DA=1, 直接地址翻译模式
    end else if (except) begin
        crmd[2:0] <= 3'b0;                          // PLV←0, IE←0
    end else if (wea && waddr == CSR_CRMD) begin
        crmd <= wdata & 32'h000001FF;
    end
end

// ============================================================
// PRMD (0x1): 例外前模式信息
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        prmd <= 32'b0;
    end else if (except) begin
        prmd <= {29'b0, crmd[2], crmd[1:0]};          // PIE←CRMD.IE, PPLV←CRMD.PLV
    end else if (wea && waddr == CSR_PRMD) begin
        prmd <= wdata & 32'h00000007;
    end
end

// ============================================================
// ECFG (0x4): 例外配置
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ecfg <= 32'b0;
    end else if (wea && waddr == CSR_ECFG) begin
        ecfg <= wdata & 32'h00001BFF;
    end
end

// ============================================================
// ESTAT (0x5): 例外状态
//   IS[1:0]  = SWI     (软件 CSR 写入)
//   IS[9:2]  = HWI     (硬件采样)
//   IS[11]   = TI      (定时器采样)
//   IS[12]   = IPI     (核间中断采样)
//   Ecode/EsubCode     (硬件异常写入)
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        estat <= 32'b0;
    end else begin
        // 硬件中断采样 (每拍)
        estat[9:2]  <= hwi;
        estat[11]   <= ti_signal;
        estat[12]   <= ipi;

        // 异常 → Ecode/EsubCode
        if (except) begin
            estat[21:16] <= except_ecode;
            estat[30:22] <= 9'b0;
        end

        // 软件中断 (CSR 指令写)
        if (wea && waddr == CSR_ESTAT)
            estat[1:0] <= wdata[1:0];
    end
end

// ============================================================
// ERA (0x6): 例外返回地址 — 全 32 位 RW
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        era <= 32'b0;
    end else if (except) begin
        era <= except_era;
    end else if (wea && waddr == CSR_ERA) begin
        era <= wdata;
    end
end

// ============================================================
// BADV (0x7): 出错虚地址 — 全 32 位 RW
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        badv <= 32'b0;
    end else if (except) begin
        badv <= except_badv;
    end else if (wea && waddr == CSR_BADV) begin
        badv <= wdata;
    end
end

// ============================================================
// EENTRY (0xC): 例外入口地址
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        eentry <= 32'b0;
    end else if (wea && waddr == CSR_EENTRY) begin
        eentry <= wdata & 32'hFFFFFFC0;
    end
end

// ============================================================
// LLBCTL (0x60): LLBit 控制
//   [0] LLbit (RW), [4] KLO (RW)
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        llbctl <= 32'b0;
    end else if (ertn && llbctl[4] != 1'b1) begin
        llbctl[0] <= 1'b0;                       // KLO≠1 → LLbit←0
    end else if (wea && waddr == CSR_LLBCTL) begin
        llbctl <= wdata & 32'h00000011;
    end
end

// ============================================================
// TID (0x40): 定时器编号 — 全 32 位 RW
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tid <= 32'b0;
    end else if (wea && waddr == CSR_TID) begin
        tid <= wdata;
    end
end

// ============================================================
// TCFG (0x41): 定时器配置
//   [0] En, [1] Periodic, [TIMER_WIDTH-1:2] InitVal
// ============================================================
wire [TIMER_WIDTH-1:0] tcfg_mask = (1 << TIMER_WIDTH) - 1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tcfg <= 32'b0;
    end else if (wea && waddr == CSR_TCFG) begin
        tcfg <= wdata & {{32-TIMER_WIDTH{1'b0}}, tcfg_mask};
    end
end

// ============================================================
// TVAL (0x42): 定时器计数值 — 只读, 由组合线输出
// ============================================================
reg  [TIMER_WIDTH-1:0] timer_val;
reg                     ti_signal;
wire [TIMER_WIDTH-1:0] tval_comb = timer_val;

// ============================================================
// 定时器计数逻辑
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        timer_val <= 0;
        ti_signal <= 1'b0;
    end else begin
        // TCFG 写入 → 重载计数器
        if (wea && waddr == CSR_TCFG)
            timer_val <= wdata[TIMER_WIDTH-1:2] << 2;
        // 计数
        else if (tcfg[0] && |timer_val)
            timer_val <= timer_val - 1;
        else if (tcfg[0]) begin
            ti_signal <= 1'b1;
            if (tcfg[1])
                timer_val <= tcfg[TIMER_WIDTH-1:2] << 2;
        end

        // TICLR 写 1 → 清 TI
        if (wea && waddr == CSR_TICLR && wdata[0])
            ti_signal <= 1'b0;
    end
end

// ============================================================
// TICLR (0x44): 定时中断清除 — 读 0, 写 1 清 TI
//   由定时器 always 块处理
// ============================================================

// ============================================================
// SAVE0~3 (0x30~0x33): 数据保存 — 全 32 位 RW
// ============================================================
genvar i;
generate
    for (i = 0; i < 4; i = i + 1) begin : gen_save
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                save[i] <= 32'b0;
            end else if (wea && waddr == (CSR_SAVE0 + i)) begin
                save[i] <= wdata;
            end
        end
    end
endgenerate

endmodule