@chcp 65001 >nul
@echo off
setlocal enabledelayedexpansion

if "%~1"=="" (
    echo 用法: asm2mem32.bat bench_core.asm
    pause
    exit /b
)

set asm_file=%~1
set base_name=%~dpn1

if not exist "%asm_file%" (
    echo 错误：找不到文件 "%asm_file%"
    pause
    exit /b
)

echo 正在汇编: %asm_file%
del "%base_name%.o" "%base_name%.elf" "%base_name%.bin" 2>nul
loongarch64-linux-gnu-as -o "%base_name%.o" "%asm_file%"
if errorlevel 1 (
    echo 汇编失败
    pause
    exit /b
)

echo 正在链接...
loongarch64-linux-gnu-ld -Ttext=0x0 -e 0 -o "%base_name%.elf" "%base_name%.o"
if errorlevel 1 (
    echo 链接失败
    pause
    exit /b
)

echo 正在生成二进制...
loongarch64-linux-gnu-objcopy -O binary "%base_name%.elf" "%base_name%.bin"
if errorlevel 1 (
    echo 生成二进制失败
    pause
    exit /b
)

echo 正在转换为 32 位 .mem 格式 (每行一条指令, 8位十六进制)...
powershell -command "$bytes = [System.IO.File]::ReadAllBytes('%base_name%.bin'); $out = @(); for ($i=0; $i -lt $bytes.Count; $i+=4) { if ($i+3 -lt $bytes.Count) { $word = [System.BitConverter]::ToUInt32($bytes, $i); $out += '{0:X8}' -f $word } else { $remaining = $bytes.Count - $i; $temp = New-Object byte[] 4; [Array]::Copy($bytes, $i, $temp, 0, $remaining); $word = [System.BitConverter]::ToUInt32($temp, 0); $out += '{0:X8}' -f $word } }; [System.IO.File]::WriteAllLines('%base_name%.mem', $out)"
if errorlevel 1 (
    echo 转换失败
    pause
    exit /b
)

echo 完成: %base_name%.mem

rem ── 清理中间文件 ──
del "%base_name%.o" "%base_name%.elf" "%base_name%.bin" 2>nul
echo 已清理中间文件
