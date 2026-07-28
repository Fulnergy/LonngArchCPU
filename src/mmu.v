// 地址翻译模块 — 简化版
//   DA=1: 直接地址翻译, pa = va
//   PG=1: 映射地址翻译, 先查 DMW, 不命中暂直通 (待 TLB)
module mmu (
    input  [31:0] if_va,          // 取指虚地址
    input  [31:0] mem_va,         // 访存虚地址
    input  [1:0]  plv,            // 当前特权等级
    input         da, pg,         // CRMD 翻译模式
    input  [31:0] dmw0, dmw1,     // DMW 窗口
    output [31:0] if_pa,          // 取指物理地址
    output [31:0] mem_pa          // 访存物理地址
);

    // DMW 命中: VA[31:29] 匹配 VSEG 且特权级允许
    function dmw_hit;
        input [31:0] va, dmw;
        begin
            dmw_hit = (va[31:29] == dmw[31:29])
                   && ( (plv == 2'd0 && dmw[0])
                     || (plv == 2'd3 && dmw[3]) );
        end
    endfunction

    function [31:0] translate;
        input [31:0] va;
        begin
            translate = da               ? va :
                        (pg && dmw_hit(va, dmw0)) ? {dmw0[27:25], va[28:0]} :
                        (pg && dmw_hit(va, dmw1)) ? {dmw1[27:25], va[28:0]} : va;
        end
    endfunction

    assign if_pa  = translate(if_va);
    assign mem_pa = translate(mem_va);

endmodule
