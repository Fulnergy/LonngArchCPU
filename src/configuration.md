目前暂未实现浮点指令、优化、流水线控制等，但为其留出了空间
## signal bus
理念：信号失效最早的放在高位，最晚被需要的放在低位
如opcode,func在EX就失效，安排在最高位，regWrite在WB要用，安排在最低位。
|jump|branch|memRead|memWrite|regWrite|
|---|---|---|---|---|
|4|3|2|1|0|

## regs address
理念：与instruction中排列顺序一致
|ra|rk|rj|rd|
|---|---|---|---|
|19:15|14:10|9:5|4:0|
