module Regs(
    input clk,
    input en,
    input regWrite0, regWrite1,
    input [4:0] write_addr0, write_addr1,     // 写入地址
    input [4:0] read_addr01, read_addr02,     // 槽0 读取地址 (rs, rt)
               read_addr11, read_addr12,      // 槽1 读取地址 (rs, rt)
    input [31:0] write_data0, write_data1,    // 写入数据
    output [31:0] read01, read02,             // 槽0 读出数据
                  read11, read12              // 槽1 读出数据
);

    // ============================================================
    // 32 x 32-bit 通用寄存器文件
    // r0 硬连线为 0 (LoongArch ABI 规定)
    // ============================================================
    reg [31:0] regs [0:31];

    // 上电初始化全部为 0 (仿真用，综合时通常被忽略)
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            regs[i] = 32'b0;
    end

    // ============================================================
    // 写端口 (时序逻辑，上升沿写入)
    // - 写使能由 regWrite 控制，受 en 总使能门控
    // - r0 不可写入，始终保持 0
    // - 同周期双写冲突：slot1 优先 (后写入覆盖)
    // ============================================================
    always @(posedge clk) begin
        if (en) begin
            if (regWrite0 && (write_addr0 != 5'b0))
                regs[write_addr0] <= write_data0;
            if (regWrite1 && (write_addr1 != 5'b0))
                regs[write_addr1] <= write_data1;
        end
    end

    // ============================================================
    // 读端口 (组合逻辑，立即返回)
    // - r0 始终返回 0
    // - 支持同一周期内写透明（先写后读需由上层转发保证）
    // ============================================================
    assign read01 = (read_addr01 == 5'b0) ? 32'b0 : regs[read_addr01];
    assign read02 = (read_addr02 == 5'b0) ? 32'b0 : regs[read_addr02];
    assign read11 = (read_addr11 == 5'b0) ? 32'b0 : regs[read_addr11];
    assign read12 = (read_addr12 == 5'b0) ? 32'b0 : regs[read_addr12];

endmodule