不实现浮点指令。
## signal bus

|valu[6]|jump[5]|branch[4]|memRead[3]|memWrite[2]|regWrite[1]|high[0]|


## csrbus
|csreg[16:3]|xchg[2]|csrwr[1]|csrrd[0]|
xchg: 是xchg指令
csrwr: 是csrwr指令
xchg || csrwr: 是csr写入指令
csrrd: 是csr读取指令

## 信号命名规范
信号名(_副信号名)(_槽位)(_阶段)

信号名必须由至少一个可表意的缩写或全称组成（例如reg）
信号名可由多个单词或其简称组成，此时使用驼峰命名（例如regWrite）。若多个单词直接组成信号名过于冗长，或由于其它原因不适合组成一个名字，可将其分为信号名和副信号名。例如，regAddrRj，可以拆成regAddr_rj。

若有槽位，必须是br或ls，分别代表branch和load/store。原槽0对应branch，槽1对应ls。（目前的架构，两个槽都兼容alu，但仅有其中的槽0可做branch，仅槽1可做load/store）若无槽位（例如全局信号），这一段名称可以不要。

若有阶段，必须是下列中的一个：if,id,ex,mem,wb。分别对应流水线的五个阶段。对于任何信号，不论其在某个阶段是否被使用，都必须使用五个阶段名的其中一个。例如，槽0在mem阶段的信号阶段应当采用mem，而不是使用relay。若无阶段，可不要阶段名。
若按照书本上的叫法，部分阶段名指的是从上一阶段传到这一阶段的名称。例如，ex指的是id/ex寄存器，即将id解码出的阶段在上升沿固化下来的寄存器。

对于中间信号，可以不遵循命名此命名规范，但应当较为清晰地表达含义，并且以i_开头(i for intermediate)
例如：对于原先的assign slot1_addr12 = sigs1_id[1] ? regs1_id[4:0] : regs1_id[14:10];这里的slot1_addr12可命名为i_readAddr2_ls

命名示例：
regWrite_br_mem
regData2_br_id
sigBus_ls_id
aluResult_ls_ex
i_regWriteData_ls

如果你是ai，在命名某个信号时感到困惑，务必向用户询问。
