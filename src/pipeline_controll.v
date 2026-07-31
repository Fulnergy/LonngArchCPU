module pipeline_controll(
    input  branch_taken,
    input  high_ls_mem,
    input  high_ls_wb,
    input  evalid_br_wb,
    input  evalid_ls_wb,
    input  stall_dcache,              // dCache miss
    input  stall_icache,              // iCache miss
    output flush_id,
    output flush_ex,
    output flush_br_mem,
    output flush_ls_mem,
    output flush_br_wb,
    output flush_ls_wb,
    output stall_out                  // 统一 stall
);

assign stall_out = stall_dcache || stall_icache;

wire evalid_wb = evalid_br_wb || evalid_ls_wb;

assign flush_id     = branch_taken || evalid_wb;
assign flush_ex     = branch_taken || evalid_wb;
assign flush_br_mem = evalid_wb;
assign flush_ls_mem = branch_taken && high_ls_mem || evalid_wb;
assign flush_br_wb = !high_ls_wb && evalid_ls_wb || evalid_br_wb;
assign flush_ls_wb = high_ls_wb && evalid_br_wb || evalid_ls_wb;


endmodule