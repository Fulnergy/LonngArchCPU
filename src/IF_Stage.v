// IF Stage — 双发射取指, 64b imem → 两条 32b 指令
module IF_Stage(
    input  wire         clk,
    input  wire         en,              // 取指使能, 0=stall
    input  wire [12:0]  inst_addr,       // 字节地址 (13b → 8KB)
    output wire [31:0]  inst_alu,        // 低 32b 指令 → ALU 流水线
    output wire [31:0]  inst_ls          // 高 32b 指令 → LS 流水线
);

    // imem 64b 数据线
    wire [63:0] imem_dout;

    // 例化 64 位宽指令存储器
    imem #(
        .ADDR_WIDTH (13),                // 13b 字节地址 → addr[12:3] → 1024 条指令对
        .DATA_WIDTH (64),
        .INIT_FILE  ("imem_init.hex")
    ) u_imem (
        .clk   (clk),
        .rst_n (1'b1),                   // BRAM 读路径无需复位
        .addr  (inst_addr),
        .dout  (imem_dout)
    );

    // 指令拆分: {inst_odd, inst_even}
    // inst_addr 低 3 位恒为 0 (8 字节对齐), imem 一次吐出相邻两条指令
    assign inst_alu = imem_dout[31:0];    // 偶数地址指令
    assign inst_ls  = imem_dout[63:32];   // 奇数地址指令 (+4)

endmodule