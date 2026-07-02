module dMem #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 15,
    parameter INIT_FILE  = "memdata.hex"
)(
    input clk,
    input writeEn,
    input [3:0] byteWe,
    input [ADDR_WIDTH-1:0] addr,
    input [DATA_WIDTH-1:0] din,
    output reg [DATA_WIDTH-1:0] dout
);

    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];

    integer i;
    initial begin
        if (INIT_FILE != "")
            $readmemh(INIT_FILE, mem);
    end

    always @(posedge clk) begin
        dout <= mem[addr];
        if (writeEn) begin
            if (byteWe[0]) mem[addr][7:0]   <= din[7:0];
            if (byteWe[1]) mem[addr][15:8]  <= din[15:8];
            if (byteWe[2]) mem[addr][23:16] <= din[23:16];
            if (byteWe[3]) mem[addr][31:24] <= din[31:24];
        end
    end

endmodule