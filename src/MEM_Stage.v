// ============================================================================
// MEM_Stage — 访存阶段, 例化 dCache 替代 dMem
//   MEM 级: 驱动 dCache 请求 (cpu_req/cpu_we/cpu_addr/cpu_wdata/cpu_wstrb)
//   WB  级: 读数据子字提取 + 符号/零扩展 (组合逻辑, 与 cpu_rdata 对齐)
//   stall: dCache cpu_stall 透传给流水线控制器
//   ext:   外部 AXI 接口透传 (接 axi_bridge)
// ============================================================================
module MEM_Stage #(
    parameter ADDR_WIDTH = 32,               // dCache 使用 32 位地址
    parameter DATA_WIDTH = 32
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // ── MEM 级 ──
    input  wire                      mem_req,        // 访存有效 (memRead || memWrite)
    input  wire                      wr_en,          // 写使能
    input  wire [1:0]                mem_size,       // 00=byte, 01=half, 10=word
    input  wire [ADDR_WIDTH-1:0]     data_addr,      // 字节地址
    input  wire [DATA_WIDTH-1:0]     write_data,     // store 数据 (低位对齐)
    input  wire [3:0]                write_strb,     // 字节写使能

    // ── WB 级 (读数据处理, 与 dCache cpu_rdata 同步) ──
    input  wire                      signExt_wb,     // 0=符号扩展, 1=零扩展
    input  wire [1:0]                mem_size_wb,    // 访存宽度 (延迟 1 拍)
    input  wire [1:0]                addr_low_wb,    // data_addr[1:0] (延迟 1 拍)

    // ── 输出 ──
    output wire [DATA_WIDTH-1:0]     read_data,      // load 读出数据 (已扩展)
    output wire                      cpu_stall,      // dCache stall → 流水线

    // ── 外部存储 (接 axi_bridge) ──
    output wire                      ext_req,
    output wire                      ext_we,
    output wire [ADDR_WIDTH-1:0]     ext_addr,
    output wire [DATA_WIDTH-1:0]     ext_wdata,
    output wire [3:0]                ext_wstrb,
    input  wire [DATA_WIDTH-1:0]     ext_rdata,
    input  wire                      ext_ready
);

    // ============================================================
    // dCache 例化 (替代 dMem)
    // ============================================================
    wire [DATA_WIDTH-1:0] cache_rdata;

    dCache #(
        .ADDR_WIDTH      (ADDR_WIDTH),
        .DATA_WIDTH      (DATA_WIDTH),
        .LINE_SIZE_BYTES (64),
        .NUM_SETS        (128),
        .NUM_WAYS        (2)
    ) u_dcache (
        .clk        (clk),
        .rst_n      (rst_n),

        .cpu_req    (mem_req),
        .cpu_we     (wr_en),
        .cpu_size   (mem_size),
        .cpu_addr   (data_addr),
        .cpu_wdata  (write_data),
        .cpu_wstrb  (write_strb),
        .cpu_rdata  (cache_rdata),
        .cpu_stall  (cpu_stall),

        .ext_req    (ext_req),
        .ext_we     (ext_we),
        .ext_addr   (ext_addr),
        .ext_wdata  (ext_wdata),
        .ext_wstrb  (ext_wstrb),
        .ext_rdata  (ext_rdata),
        .ext_ready  (ext_ready)
    );

    // ============================================================
    // WB 级: 读数据处理 (组合逻辑, cache_rdata 已延迟 1 拍)
    //   dCache 读命中: S_IDLE(stall=0) → S_DATA(cpu_rdata 有效)
    //   缺失时 stall=1, WB 级寄存器冻结, 直到数据就绪
    // ============================================================
    wire [3:0] byte_we_wb;
    assign byte_we_wb =
        (mem_size_wb == 2'b00) ? (4'b0001 << addr_low_wb) :
        (mem_size_wb == 2'b01) ? (4'b0011 << {addr_low_wb[1], 1'b0}) :
                                  4'b1111;

    wire [DATA_WIDTH-1:0] byte_mask;
    assign byte_mask = {{8{byte_we_wb[3]}}, {8{byte_we_wb[2]}},
                        {8{byte_we_wb[1]}}, {8{byte_we_wb[0]}}};

    wire [DATA_WIDTH-1:0] masked_data;
    assign masked_data = cache_rdata & byte_mask;

    wire [4:0] shift_amt_wb;
    assign shift_amt_wb = byte_we_wb[0] ? 5'd0  :
                          byte_we_wb[1] ? 5'd8  :
                          byte_we_wb[2] ? 5'd16 :
                                           5'd24;

    wire [DATA_WIDTH-1:0] aligned_data;
    assign aligned_data = masked_data >> shift_amt_wb;

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