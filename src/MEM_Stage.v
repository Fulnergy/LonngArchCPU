// 数据存储器 —— 例化 dMem, 内部译码 byte_we, signExt 控制符号/零扩展
//   MEM 级信号 → dmem 端口 (posedge 采样)
//   WB  级信号 → 读数据处理 (组合逻辑, 与 dmem 读输出对齐)
module MEM_Stage #(
    parameter ADDR_WIDTH = 17,               // 字节地址位宽 (17b → 128KB)
    parameter DATA_WIDTH = 32,               // 数据位宽
    parameter INIT_FILE  = "dmem_init.hex"   // 仿真初始化文件
)(
    input  wire                      clk,
    // ── MEM 级 (dmem posedge 采样) ──
    input  wire                      wr_en,          // 写使能
    input  wire [1:0]                mem_size,       // 访存宽度
    input  wire [ADDR_WIDTH-1:0]     data_addr,      // 字节地址
    input  wire [DATA_WIDTH-1:0]     write_data,     // store 数据 (已低位对齐)
    // ── WB 级 (读数据处理, 与 dmem_dout 同步) ──
    input  wire                      signExt_wb,     // 0=符号扩展, 1=零扩展
    input  wire [1:0]                mem_size_wb,    // 访存宽度 (延迟1拍)
    input  wire [1:0]                addr_low_wb,    // data_addr[1:0] (延迟1拍)
    // ── 输出 ──
    output wire [DATA_WIDTH-1:0]     read_data       // load 读出数据 (已扩展)
);

    localparam WORD_WIDTH = ADDR_WIDTH - 2;  // 字地址位宽 (17→15)

    // ============================================================
    // MEM 级: byte_we & 写数据对齐 (用于 dmem posedge 采样)
    // ============================================================
    wire [3:0] byte_we_mem;
    assign byte_we_mem =
        (mem_size == 2'b00) ? (4'b0001 << data_addr[1:0]) :
        (mem_size == 2'b01) ? (4'b0011 << {data_addr[1], 1'b0}) :
                               4'b1111;

    wire [4:0] shift_amt_mem;
    assign shift_amt_mem = byte_we_mem[0] ? 5'd0  :
                           byte_we_mem[1] ? 5'd8  :
                           byte_we_mem[2] ? 5'd16 :
                                            5'd24;

    // 写数据 → 内存字内对应字节位置 (左移)
    wire [DATA_WIDTH-1:0] aligned_wdata;
    assign aligned_wdata = write_data << shift_amt_mem;

    // 字节地址 → 字地址
    wire [WORD_WIDTH-1:0] word_addr;
    assign word_addr = data_addr[ADDR_WIDTH-1:2];

    // ============================================================
    // dMem BRAM (同步读, posedge 寄存)
    // ============================================================
    wire [DATA_WIDTH-1:0] dmem_dout_raw;

    dMem #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (WORD_WIDTH),
        .INIT_FILE  (INIT_FILE)
    ) u_dmem (
        .clk     (clk),
        .writeEn (wr_en),
        .byteWe  (byte_we_mem),
        .addr    (word_addr),
        .din     (aligned_wdata),
        .dout    (dmem_dout_raw)
    );

    // ============================================================
    // WB 级: 读数据处理 (组合逻辑, dmem_dout_raw 已延迟 1 拍)
    // ============================================================
    wire [3:0] byte_we_wb;
    assign byte_we_wb =
        (mem_size_wb == 2'b00) ? (4'b0001 << addr_low_wb) :
        (mem_size_wb == 2'b01) ? (4'b0011 << {addr_low_wb[1], 1'b0}) :
                                  4'b1111;

    // 字节掩码
    wire [DATA_WIDTH-1:0] byte_mask;
    assign byte_mask = {{8{byte_we_wb[3]}}, {8{byte_we_wb[2]}}, {8{byte_we_wb[1]}}, {8{byte_we_wb[0]}}};

    wire [DATA_WIDTH-1:0] masked_data;
    assign masked_data = dmem_dout_raw & byte_mask;

    // 右移对齐到 reg 低位
    wire [4:0] shift_amt_wb;
    assign shift_amt_wb = byte_we_wb[0] ? 5'd0  :
                          byte_we_wb[1] ? 5'd8  :
                          byte_we_wb[2] ? 5'd16 :
                                           5'd24;

    wire [DATA_WIDTH-1:0] aligned_data;
    assign aligned_data = masked_data >> shift_amt_wb;

    // 符号/零扩展
    wire [2:0] byte_cnt;
    assign byte_cnt = byte_we_wb[0] + byte_we_wb[1] + byte_we_wb[2] + byte_we_wb[3];

    wire [4:0] data_msb;
    assign data_msb = (&byte_we_wb)   ? 5'd31 :
                      byte_cnt[1]     ? 5'd15 :
                                         5'd7;

    wire [31:0] upper_mask;
    assign upper_mask = 32'hFFFFFFFF << (data_msb + 1);

    wire [31:0] sign_fill;
    assign sign_fill = {32{aligned_data[data_msb]}} & upper_mask;

    assign read_data = signExt_wb ? aligned_data : (aligned_data | sign_fill);

endmodule