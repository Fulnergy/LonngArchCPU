// IF Stage — 双发射取指, 64b imem → 两条 32b 指令
module IF_Stage #(
    parameter IMEM_FILE = "imem_init.hex"
)
(
    input  wire         clk,
    input  wire         en,              // 取指使能, 0=stall
    input  wire [12:0]  pc,       // 字节地址 (13b → 8KB)
    output wire [63:0]  dual_inst        // 双发射指令: {inst_ls, inst_alu}
);

    // 例化 64 位宽指令存储器
    imem #(
        .ADDR_WIDTH (13),                // 13b 字节地址 → addr[12:3] → 1024 条指令对
        .DATA_WIDTH (64),
        .INIT_FILE  (IMEM_FILE)
    ) u_imem (
        .clk   (clk),
        .rst_n (1'b1),                   // BRAM 读路径无需复位
        .addr  (pc),
        .dout  (dual_inst)
    );


endmodule