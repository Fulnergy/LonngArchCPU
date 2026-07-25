module inst_controll(
    input clk,
    input rst_n,
    input jump_taken,
    input except_taken,                 // 异常跳转 (最高优先级)
    input [31:0] pc_except,            // 异常入口地址
    input nopl, noph, //出于发射顺序考虑，若低pc不发射，高pc也不发射
    input [31:0] pc_jump,
    input [63:0] dual_inst_raw,//来自IF的生指令
    output reg [31:0] pc_next,//将给IF取指的地址
    output wire [31:0] pc_low,//本次发射的指令中，较低那条的pc
    output [63:0] dual_inst //输出给ID的指令
);

parameter NOP = 32'h03400000;

reg [31:0] left;//之前剩的指令，若没剩则不需要有
reg take;//是否使用之前剩的
reg [31:0] pc_last;//当前拍的dual_inst_raw中低位指令对应的pc
//在当前架构下，双指令理应8byte对齐，但跳转时目标仅确保4byte对齐
wire misaligned = pc_last[2];
//仅一种情况会未准备好：跳转之后，目标pc是双指令的高位。
//此时没有对齐，输入的dual_inst的低位是无效的。
//解决方案是，将输入指令的高位作为输出指令的低位，并将高位填充nop

//pc控制
always @(*) begin
    if(!rst_n)begin
        pc_next=32'b0;
    end
    else if(except_taken)begin
        pc_next=pc_except;
    end
    else if(jump_taken)begin
        pc_next=pc_jump;
    end
    //都不发射，或这次只要一条，且正好之前有剩的一条，则不需要往后取
    else if(nopl||noph&&take)begin
        pc_next=pc_last;
    end
    else if(misaligned)begin
        pc_next=pc_last+4;
    end
    //否则向后取两条指令
    else begin
        pc_next=pc_last+8;
    end
end

always @(posedge clk) begin
    if(~rst_n)begin
        pc_last<=32'b0;
    end
    else begin
        pc_last<=pc_next;
    end
end

// ============================================================
// 信号裁定 (决定下一拍的 dual_inst 拼接方式)
// ============================================================
// 调度约束: 必定是低位指令运行，高位被保存
// ============================================================
always @(posedge clk) begin
    if(~rst_n)begin
        take<=1'b0;
        left<=NOP;
    end
    else if(jump_taken || except_taken) begin
        take<=1'b0;
        left<=NOP;
    end
    else if(nopl) begin
        take<=take;
        left<=left;
    end
    //之前剩一条，现在取两条 或 之前没剩，现在取一条
    else if((take&&!noph)||(!take&&noph)) begin
        take<=1'b1;
        left<=dual_inst_raw[63:32];
    end
    else begin
        take<=1'b0;
        left<=NOP;
    end
end

assign dual_inst = misaligned ? {NOP,dual_inst_raw[63:32]} :
                   take       ? {dual_inst_raw[31:0],left} : dual_inst_raw;
//实际运行时，这一拍的双指令按规则取出并给到ID
//随后ID解出的结果(nop)再返回本模块，通过上述always块决定下一拍如何解

assign pc_low = take ? pc_last - 4 : pc_last;

endmodule