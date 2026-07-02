module imem #(
    parameter ADDR_WIDTH = 10, // Address width (10 bits for 1024 bytes)
    parameter DATA_WIDTH = 32,  // Data width (32 bits)
    parameter INIT_FILE = "imem_init.hex" // Initialization file for simulation
)(
    input wire clk,
    input wire rst_n,
    input wire [ADDR_WIDTH-1:0] addr,
    output reg [DATA_WIDTH-1:0] dout
);
    
    // BRAM storage
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];

    // BRAM initialization (for simulation; maps to INIT values in synthesis)
    initial begin
        $readmemh(INIT_FILE, mem);
    end

    // Synchronous BRAM read: addr → dout, 1-cycle latency
    // NO reset on dout — BRAM output registers do not support async reset
    always @(posedge clk) begin
        dout <= mem[addr[ADDR_WIDTH-1:2]];
    end

endmodule