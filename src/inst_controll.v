module inst_controll(
    input clk,
    input rst_n,
    input jump_taken,
    input nopl, noph, //出于发射顺序考虑，若低pc不发射，高pc也不发射
    input [31:0] pc_jump,
    input [63:0] dual_inst_raw,//来自IF的生指令
    output reg [31:0] pc_next,//将给IF取指的地址
    output wire [31:0] pc_low,//本次发射的指令中，较低那条的pc
    output [63:0] dual_inst //输出给ID的指令
);

reg [31:0] left;//之前剩的指令，若没剩则不需要有
reg take;//是否使用之前剩的
reg [31:0] pc_last;

//pc控制
always @(*) begin
    if(!rst_n)begin
        pc_next=32'b0;
    end
    else if(jump_taken)begin
        pc_next=pc_jump;
    end
    //都不发射，或这次只要一条，且正好之前有剩的一条，则不需要往后取
    else if(nopl||noph&&take)begin
        pc_next=pc_last;
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
// take / left 状态机 (决定下一拍的 dual_inst 拼接方式)
// ============================================================
// 调度约束: 仅当双branch或双ls时，只取一条指令
//          必定是低位指令运行，高位被保存
// ============================================================
always @(posedge clk) begin
    if(~rst_n)begin
        take<=1'b0;
        left<=32'b0;
    end
    else if(jump_taken) begin
        take<=1'b0;
        left<=32'b0;
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
        left<=32'b0;
    end
end

assign dual_inst = take ? {dual_inst_raw[31:0],left} : dual_inst_raw;
//实际运行时，这一拍的双指令按规则取出并给到ID
//随后ID解出的结果(nop)再返回本模块，通过上述always块决定下一拍如何解

assign pc_low = take ? pc_last - 4 : pc_last;

endmodule