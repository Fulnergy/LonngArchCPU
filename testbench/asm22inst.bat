@chcp 65001 >nul
@echo off
setlocal enabledelayedexpansion

if "%~1"=="" (
    echo 用法：请将 .asm 文件拖拽到本脚本图标上
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
loongarch64-linux-gnu-as -o "%base_name%.o" "%asm_file%"
if errorlevel 1 (
    echo ❌ 汇编失败，请检查语法错误。
    pause
    exit /b
)

echo 正在链接...
loongarch64-linux-gnu-ld -Ttext=0x0 -e 0 -o "%base_name%.elf" "%base_name%.o"
if errorlevel 1 (
    echo ❌ 链接失败，请检查符号或地址设置。
    pause
    exit /b
)

echo 正在生成二进制文件...
loongarch64-linux-gnu-objcopy -O binary "%base_name%.elf" "%base_name%.bin"
if errorlevel 1 (
    echo ❌ 生成二进制失败！
    pause
    exit /b
)

echo 正在转换为 64 位双发射格式（每行 16 位十六进制）...
powershell -command "$bytes = [System.IO.File]::ReadAllBytes('%base_name%.bin'); $out = @(); for ($i=0; $i -lt $bytes.Count; $i+=8) { if ($i+7 -lt $bytes.Count) { $word = [System.BitConverter]::ToUInt64($bytes, $i); $out += '{0:X16}' -f $word } else { $remaining = $bytes.Count - $i; $temp = New-Object byte[] 8; [Array]::Copy($bytes, $i, $temp, 0, $remaining); $word = [System.BitConverter]::ToUInt64($temp, 0); $out += '{0:X16}' -f $word } }; [System.IO.File]::WriteAllLines('%base_name%.mem', $out)"
if errorlevel 1 (
    echo ❌ 转换失败！
    pause
    exit /b
)

echo 清理中间文件...
del "%base_name%.o" "%base_name%.elf" "%base_name%.bin" 2>nul

echo ✅ 成功！双发射机器码文件已生成: "%base_name%.mem"
echo 每行 16 位十六进制数，对应一个 64 位内存单元（低地址指令在低 32 位）。
pause
endlocal