module pipeline_controll(
    input  branch_taken,
    input  high_ls_mem,
    output flush_id,
    output flush_ex,
    output flush_ls_mem
);

assign flush_id     = branch_taken;
assign flush_ex     = branch_taken;
assign flush_ls_mem = branch_taken && high_ls_mem;

endmodule