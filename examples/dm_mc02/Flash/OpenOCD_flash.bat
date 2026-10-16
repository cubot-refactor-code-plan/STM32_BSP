@echo off
REM 工程名取自 CMake 缓存（CMAKE_PROJECT_NAME），不再依赖目录名；还没配置过时退回目录名
for %%i in ("%~dp0\..") do set "PROJECT_NAME=%%~nxi"
for /f "tokens=2 delims==" %%a in ('findstr /b "CMAKE_PROJECT_NAME:STATIC=" "%~dp0\..\build\Debug\CMakeCache.txt" 2^>nul') do set "PROJECT_NAME=%%a"

REM 构建 ELF 文件路径（使用正斜杠避免 OpenOCD 路径问题）
set "ELF_FILE=build/debug/%PROJECT_NAME%.elf"
set "ELF_FILE=%ELF_FILE:\=/%"

REM 检查文件是否存在
if not exist "%ELF_FILE%" (
    echo Error: ELF file not found at %ELF_FILE%
    exit /b 1
)

REM 使用标准化路径烧录（禁用 GDB / Telnet / TCL 服务端口）
openocd -f Flash/daplink.cfg ^
    -c "gdb_port disabled" ^
    -c "tcl_port disabled" ^
    -c "telnet_port disabled" ^
    -c "program \"%ELF_FILE%\" verify reset exit"