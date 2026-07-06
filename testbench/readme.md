https://ftp.loongnix.cn/toolchain/gcc/release/loongarch/gcc8/loongson-gnu-toolchain-8.3-i686-mingw-loongarch64-linux-gnu-rc1.6.zip
访问这个链接，把下载的工具链解压后，将其中的bin文件夹加入系统PATH，比方说C:\loongson-gnu-toolchain-8.3-i686-mingw-loongarch64-linux-gnu-rc1.6\bin

然后就可以使用汇编脚本了。

使用时，将asm文件选中，拖到bat文件上（用asm22inst.bat打开），或使用.\asm22inst.bat bench_simple.asm 这样的命令即可。