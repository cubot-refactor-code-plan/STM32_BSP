#!/usr/bin/env python3
"""
stm32_bsp 宿主工程自动接入：按 README 第 1 节把 CubeMX（CMake）工程改成能用本库的样子。

用法（在宿主工程根目录，或用 --host 指定）：

    python3 third_party/stm32_bsp/tools/integrate.py --dry-run   # 只检查并打印将要做的修改
    python3 third_party/stm32_bsp/tools/integrate.py             # 执行

做的事（每一步都先判断是否已完成，已完成就跳过，可重复运行）：
  1. 检查 CubeMX 前提（README 1.1）：STM32H7、CMake 工具链、FreeRTOS CMSIS_V2、tick 1 kHz、栈溢出检查 2、分配失败钩子
  2. 链接脚本加 .dma_buffer / .noinit（templates/linker_sections.ld）
  3. FreeRTOSConfig.h 的 USER CODE 1：configASSERT → fault_on_assert（templates/FreeRTOSConfig_user_code.h）
  4. freertos.c 的 Includes / 4 / 5 / Init 段：钩子记录现场、调 app_init()（templates/freertos_user_code.c）
  5. 复制 board_cfg.* / app_main.* 到应用目录（已存在的不覆盖），并对照本工程核对模板用到的句柄与参数
  6. 复制 hardware_config.h 到 Core/Inc/（已存在的不覆盖）
  7. 根 CMakeLists.txt 末尾接入库与应用源文件
  8. .vscode/settings.json 配置 clangd 的编译数据库目录与 query-driver

所有片段都从 templates/ 读取，与模板保持一致。只改内容仍是 CubeMX 默认的 USER CODE 段；遇到已有自定义内容
就报错并说明要手动合并的位置。任何一步报错都不写任何文件。保持各文件原有的行尾（CRLF / LF）。
"""

import argparse
import difflib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

LIB_ROOT = Path(__file__).resolve().parent.parent
TEMPLATES = LIB_ROOT / "templates"
APP_TEMPLATES = ("board_cfg.hpp", "board_cfg.cpp", "app_main.h", "app_main.cpp")


# ==================== 报告 ====================


class Report:
    def __init__(self):
        self.items = []

    def add(self, level, step, msg):
        self.items.append((level, step, msg))

    def ok(self, step, msg):
        self.add("完成", step, msg)

    def skip(self, step, msg):
        self.add("跳过", step, msg)

    def warn(self, step, msg):
        self.add("注意", step, msg)

    def error(self, step, msg):
        self.add("错误", step, msg)

    @property
    def has_error(self):
        return any(level == "错误" for level, _, _ in self.items)

    def print(self):
        for level, step, msg in self.items:
            print(f"[{level}] {step}: {msg}")


# ==================== 文件读写（保持行尾） ====================


class TextFile:
    """读入时统一成 \\n 处理，写回时恢复原行尾"""

    def __init__(self, path):
        self.path = path
        raw = path.read_bytes()
        self.crlf = b"\r\n" in raw
        self.original = raw.decode("utf-8").replace("\r\n", "\n")
        self.text = self.original

    def to_bytes(self):
        text = self.text.replace("\n", "\r\n") if self.crlf else self.text
        return text.encode("utf-8")


class Plan:
    """先在内存里算出所有修改，确认无错误后再一次写盘"""

    def __init__(self):
        self.edits = {}  # path -> TextFile
        self.new_files = {}  # path -> str

    def edit(self, tf):
        self.edits[tf.path] = tf

    def create(self, path, text):
        self.new_files[path] = text

    def empty(self):
        return not any(tf.text != tf.original for tf in self.edits.values()) and not self.new_files

    def show(self, host):
        for path, tf in self.edits.items():
            if tf.text == tf.original:
                continue
            rel = path.relative_to(host).as_posix()
            sys.stdout.writelines(
                difflib.unified_diff(
                    tf.original.splitlines(keepends=True),
                    tf.text.splitlines(keepends=True),
                    f"a/{rel}",
                    f"b/{rel}",
                )
            )
        for path in self.new_files:
            print(f"+++ 新建 {path.relative_to(host).as_posix()}")

    def apply(self):
        for tf in self.edits.values():
            if tf.text != tf.original:
                tf.path.write_bytes(tf.to_bytes())
        for path, text in self.new_files.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding="utf-8")


# ==================== 文本工具 ====================


def user_code(text, name):
    """返回 /* USER CODE BEGIN name */ 与 END 之间内容的 (起, 止)：从 BEGIN 下一行开始，到 END 所在行行首为止"""
    begin = f"/* USER CODE BEGIN {name} */"
    end = f"/* USER CODE END {name} */"
    if text.count(begin) != 1 or text.count(end) != 1:
        return None
    start = text.index("\n", text.index(begin)) + 1
    stop = text.rfind("\n", 0, text.index(end)) + 1
    return start, stop


def section_text(text, name):
    span = user_code(text, name)
    return None if span is None else text[span[0] : span[1]]


def replace_section(text, name, content):
    start, stop = user_code(text, name)
    return text[:start] + content + text[stop:]


def strip_comments(code):
    code = re.sub(r"/\*.*?\*/", "", code, flags=re.S)
    return re.sub(r"//[^\n]*", "", code)


def compact(code):
    """去注释、去空白，用于判断一段代码是不是 CubeMX 默认内容"""
    return re.sub(r"\s+", "", strip_comments(code))


def read_template(name):
    return (TEMPLATES / name).read_text(encoding="utf-8").replace("\r\n", "\n")


def eval_int(expr):
    """CubeMX 写的 PSC / ARR 形如 24-1、2000-1、10000"""
    expr = expr.strip()
    if not re.fullmatch(r"[\d\s+\-]+", expr):
        return None
    total = 0
    for sign, num in re.findall(r"([+-]?)\s*(\d+)", expr):
        total += -int(num) if sign == "-" else int(num)
    return total


# ==================== 1. 前提检查 ====================


def check_prerequisites(host, rep):
    step = "前提"
    ok = True
    if not (host / "cmake/stm32cubemx/CMakeLists.txt").is_file():
        rep.error(step, "没有 cmake/stm32cubemx/CMakeLists.txt：CubeMX 的 Project Manager → Toolchain 要选 CMake")
        ok = False
    root_cmake = host / "CMakeLists.txt"
    if not root_cmake.is_file() or not re.search(
        r"add_subdirectory\(\s*cmake/stm32cubemx\s*\)", root_cmake.read_text(encoding="utf-8")
    ):
        rep.error(step, "根 CMakeLists.txt 里没有 add_subdirectory(cmake/stm32cubemx)")
        ok = False
    if not (host / "Core/Inc/stm32h7xx_hal_conf.h").is_file():
        rep.error(step, "没有 Core/Inc/stm32h7xx_hal_conf.h：本库只支持 STM32H7")
        ok = False

    cfg = host / "Core/Inc/FreeRTOSConfig.h"
    rtos = host / "Core/Src/freertos.c"
    if not cfg.is_file() or not rtos.is_file():
        rep.error(step, "没有 Core/Inc/FreeRTOSConfig.h 或 Core/Src/freertos.c：CubeMX 里要启用 FreeRTOS（CMSIS_V2）")
        return False
    cfg_text = cfg.read_text(encoding="utf-8")

    def define(name):
        m = re.search(rf"^\s*#define\s+{name}\s+(.+?)\s*$", cfg_text, re.M)
        return m.group(1) if m else None

    tick = define("configTICK_RATE_HZ")
    if tick is None or not re.search(r"\b1000\b", tick):
        rep.error(step, f"configTICK_RATE_HZ = {tick}：库要求 1000（CubeMX FreeRTOS → TICK_RATE_HZ）")
        ok = False
    if define("configCHECK_FOR_STACK_OVERFLOW") != "2":
        rep.error(step, "configCHECK_FOR_STACK_OVERFLOW 不是 2：CubeMX FreeRTOS → CHECK_FOR_STACK_OVERFLOW = Option2")
        ok = False
    if define("configUSE_MALLOC_FAILED_HOOK") != "1":
        rep.error(step, "configUSE_MALLOC_FAILED_HOOK 不是 1：CubeMX FreeRTOS → USE_MALLOC_FAILED_HOOK = Enabled")
        ok = False
    if '#include "cmsis_os2.h"' not in rtos.read_text(encoding="utf-8"):
        rep.error(step, "freertos.c 没有包含 cmsis_os2.h：FreeRTOS 接口要选 CMSIS_V2")
        ok = False

    if not (host / "Core/Src/stm32h7xx_hal_timebase_tim.c").is_file():
        rep.warn(step, "没找到 stm32h7xx_hal_timebase_tim.c：HAL 时基应选一个空闲定时器（SysTick 留给 FreeRTOS）")
    main_c = host / "Core/Src/main.c"
    if main_c.is_file() and "SCB_EnableDCache" in main_c.read_text(encoding="utf-8"):
        rep.warn(step, "D-Cache 已开启：库假定 D-Cache 关闭，否则 .dma_buffer 要用 MPU 设成非缓存区")
    if ok:
        rep.ok(step, "CubeMX 配置满足 README 1.1")
    return ok


# ==================== 2. 链接脚本 ====================

AXI_SRAM_ORIGIN = 0x24000000                                     # 所有 STM32H7 的 AXI SRAM（D1 域）起始地址
DMA_UNREACHABLE = ((0x00000000, 0x00040000), (0x20000000, 0x20020000))  # ITCM（最大 256K）/ DTCM：DMA1 / DMA2 访问不到


def find_linker_script(host, rep, explicit):
    if explicit:
        path = (host / explicit).resolve()
        if not path.is_file():
            rep.error("链接脚本", f"{explicit} 不存在")
            return None
        return path
    found = set()
    for cmake in sorted((host / "cmake").glob("*.cmake")):
        for m in re.finditer(r'-T\s*\\?"?([^"\s\\]+\.ld)', cmake.read_text(encoding="utf-8")):
            p = m.group(1).replace("${CMAKE_SOURCE_DIR}", str(host)).replace("${CMAKE_CURRENT_SOURCE_DIR}", str(host))
            found.add(Path(p).resolve())
    if not found:
        found = {p.resolve() for p in host.glob("*.ld")}
    if len(found) != 1:
        rep.error("链接脚本", f"无法确定链接脚本（找到 {len(found)} 个），用 --linker-script 指定")
        return None
    return found.pop()


def plan_linker(host, plan, rep, explicit, region):
    step = "链接脚本"
    path = find_linker_script(host, rep, explicit)
    if path is None:
        return
    tf = TextFile(path)
    rel = path.relative_to(host).as_posix()
    missing = [s for s in ("dma_buffer", "noinit") if not re.search(rf"\.{s}\s*(\(NOLOAD\))?\s*:", tf.text)]
    if not missing:
        rep.skip(step, f"{rel} 已有 .dma_buffer / .noinit")
        return

    regions = {
        m.group(1): int(m.group(2), 0)
        for m in re.finditer(r"^\s*([A-Za-z_]\w*)\s*\([rwxaRWXA!]+\)\s*:\s*ORIGIN\s*=\s*(0[xX][0-9A-Fa-f]+|\d+)", tf.text, re.M)
    }
    names = ", ".join(f"{n}@0x{o:08X}" for n, o in regions.items())
    if region is None:
        # 按地址认 AXI SRAM，不按名字：各型号 CubeMX 链接脚本的叫法不同（H723 RAM_D1，H743 / H750 RAM）
        region = next((n for n, o in regions.items() if o == AXI_SRAM_ORIGIN), None)
        region = region or next((r for r in ("RAM_D1", "AXI_SRAM") if r in regions), None)
        if region is None:
            rep.error(step, f"MEMORY 里没有起始于 0x{AXI_SRAM_ORIGIN:08X} 的 AXI SRAM（有 {names}），用 --ram-region 指定 DMA 可达的区")
            return
    elif region not in regions:
        rep.error(step, f"MEMORY 里没有 {region}（有 {names}）")
        return
    if any(lo <= regions[region] < hi for lo, hi in DMA_UNREACHABLE):
        rep.error(step, f"{region}@0x{regions[region]:08X} 是 ITCM / DTCM，DMA1 / DMA2 访问不到，换一个区（有 {names}）")
        return

    template = re.sub(r"^\s*/\*.*?\*/\s*\n", "", read_template("linker_sections.ld"), count=1, flags=re.S)
    blocks = {}
    for m in re.finditer(r"^[ \t]*\.(\w+)[^\n]*:\s*\{.*?\}\s*>\s*\w+", template, re.M | re.S):
        blocks[m.group(1)] = re.sub(r">\s*\w+$", f">{region}", m.group(0))

    lines = tf.text.split("\n")
    data = next((i for i, l in enumerate(lines) if re.match(r"\s*\.data\s*:", l)), None)
    if data is None:
        rep.error(step, f"{rel} 里没找到 .data 段")
        return
    # 从 .data 往回跳过空行、注释和 _sidata = LOADADDR(.data)，落到上一个输出段的结尾 "} >FLASH"
    j = data - 1
    while j >= 0:
        s = lines[j].strip()
        if s == "" or s.startswith(("/*", "*")) or s.endswith("*/") or "LOADADDR(.data)" in s:
            j -= 1
        else:
            break
    if j < 0 or "}" not in lines[j]:
        rep.error(step, f"{rel} 里 .data 之前的结构不认识，请按 templates/linker_sections.ld 手动添加")
        return

    insert = ["", "  /* stm32_bsp：见 templates/linker_sections.ld（.dma_buffer 放 BspUart 实例，.noinit 放 fault_record） */"]
    for i, name in enumerate(missing):
        if i:
            insert.append("")
        insert.extend(blocks[name].split("\n"))
    lines[j + 1 : j + 1] = insert
    tf.text = "\n".join(lines)
    plan.edit(tf)
    rep.ok(step, f"{rel} 加入 {' / '.join('.' + s for s in missing)}（>{region}，0x{regions[region]:08X}）")


# ==================== 3. FreeRTOSConfig.h ====================

DEFAULT_ASSERT = compact("#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}")


def plan_freertos_config(host, plan, rep):
    step = "FreeRTOSConfig.h"
    tf = TextFile(host / "Core/Inc/FreeRTOSConfig.h")
    if "fault_on_assert" in tf.text:
        rep.skip(step, "configASSERT 已接到 fault_on_assert")
        return
    current = section_text(tf.text, "1")
    if current is None:
        rep.error(step, "没找到唯一的 USER CODE 1 段")
        return
    if compact(current) not in ("", DEFAULT_ASSERT):
        rep.error(step, "USER CODE 1 已有自定义内容，请手动合并 templates/FreeRTOSConfig_user_code.h")
        return
    tf.text = replace_section(tf.text, "1", section_text(read_template("FreeRTOSConfig_user_code.h"), "1"))
    plan.edit(tf)
    rep.ok(step, "USER CODE 1：configASSERT → fault_on_assert()")


# ==================== 4. freertos.c ====================


def plan_freertos_c(host, plan, rep):
    step = "freertos.c"
    tf = TextFile(host / "Core/Src/freertos.c")
    tpl = read_template("freertos_user_code.c")
    done = []

    for name in ("Includes", "4", "5", "Init"):
        if section_text(tf.text, name) is None:
            rep.error(step, f"没找到唯一的 USER CODE {name} 段")
            return

    # Includes / Init：没有就追加（保留段里已有的内容）
    for name, marker in (("Includes", '#include "app_main.h"'), ("Init", "app_init(")):
        current = section_text(tf.text, name)
        if marker in current:
            continue
        add = section_text(tpl, name)
        tf.text = replace_section(tf.text, name, add if current.strip() == "" else current + add)
        done.append(name)

    # 4 / 5：钩子函数体仍是 CubeMX 生成的空函数（只有注释）才替换函数体，保留原签名
    for name, func, marker in (
        ("4", "vApplicationStackOverflowHook", "fault_on_stack_overflow"),
        ("5", "vApplicationMallocFailedHook", "fault_on_malloc_failed"),
    ):
        current = section_text(tf.text, name)
        if marker in current:
            continue
        pattern = rf"(void\s+{func}\s*\([^)]*\))\s*\{{(.*?)\n\}}"
        m = re.search(pattern, current, re.S)
        if m is None or compact(m.group(2)) != "":
            rep.error(step, f"USER CODE {name} 里的 {func}() 不是 CubeMX 默认的空函数，请手动加入 {marker}()")
            return
        body = re.search(pattern, section_text(tpl, name), re.S).group(2)
        current = current[: m.start()] + m.group(1) + "\n{" + body + "\n}" + current[m.end() :]
        tf.text = replace_section(tf.text, name, current)
        done.append(name)

    if done:
        plan.edit(tf)
        rep.ok(step, f"USER CODE {' / '.join(done)}：钩子调 fault_on_*()，MX_FREERTOS_Init() 调 app_init()")
    else:
        rep.skip(step, "钩子与 app_init() 已接好")


# ==================== 5. 应用模板 ====================


def host_handles(host):
    handles = {}
    for src in (host / "Core/Src").glob("*.c"):
        for m in re.finditer(r"^\s*\w+_HandleTypeDef\s+(h\w+)\s*;", src.read_text(encoding="utf-8"), re.M):
            handles[m.group(1)] = src
    return handles


def check_board_cfg(host, rep, board_cfg_text, app_rel):
    """模板 board_cfg.cpp 写的是 DM-MC02 的句柄与参数，对照本工程 CubeMX 生成的代码核对"""
    step = "board_cfg"
    handles = host_handles(host)
    used = sorted(set(re.findall(r"&(h\w+)", strip_comments(board_cfg_text))))
    missing = [h for h in used if h not in handles]
    if missing:
        rep.warn(step, f"模板用到 {', '.join(missing)}，本工程没有：在 {app_rel}/board_cfg.* 里删掉或换成实际句柄（连同 #include）")

    for h, baud in re.findall(r"BspUart<\d+>\s+\w+\(\{\s*&(huart\d+)\s*,\s*\w+\s*,\s*(\d+)U?\s*\}\)", board_cfg_text):
        if h in handles:
            m = re.search(rf"{h}\.Init\.BaudRate\s*=\s*(\d+)\s*;", handles[h].read_text(encoding="utf-8"))
            if m and m.group(1) != baud:
                rep.warn(step, f"{h} 模板波特率 {baud}，CubeMX 是 {m.group(1)}：改 {app_rel}/board_cfg.cpp（必须一致）")

    pwm = r"BspPwm\s+\w+\(\{\s*&(htim\d+)\s*,\s*\w+\s*,\s*(\d+)UL?\s*,\s*(\d+)U?\s*,\s*(\d+)U?\s*\}\)"
    for h, clk, psc, arr in re.findall(pwm, board_cfg_text):
        if h not in handles:
            continue
        src = handles[h].read_text(encoding="utf-8")
        for field, want in (("Prescaler", psc), ("Period", arr)):
            m = re.search(rf"{h}\.Init\.{field}\s*=\s*([^;]+);", src)
            have = eval_int(m.group(1)) if m else None
            if have is not None and have != int(want):
                rep.warn(step, f"{h} 模板 {field} = {want}，CubeMX 是 {have}：改 {app_rel}/board_cfg.cpp")
        rep.warn(step, f"{h} 的定时器时钟模板写的是 {clk} Hz（DM-MC02），按本工程时钟树核对")


def plan_app(host, plan, rep, app_dir):
    step = "应用模板"
    app = host / app_dir
    app_rel = app.relative_to(host).as_posix()
    copied = []
    for name in APP_TEMPLATES:
        if (app / name).exists():
            continue
        plan.create(app / name, (TEMPLATES / name).read_text(encoding="utf-8"))
        copied.append(name)
    if copied:
        rep.ok(step, f"复制 {', '.join(copied)} → {app_rel}/")
    else:
        rep.skip(step, f"{app_rel}/ 里已有 {', '.join(APP_TEMPLATES)}")
    if "board_cfg.cpp" in copied:
        check_board_cfg(host, rep, read_template("board_cfg.cpp"), app_rel)


# ==================== 6. hardware_config.h ====================


def plan_hardware_config(host, plan, rep):
    step = "hardware_config.h"
    dst = host / "Core/Inc/hardware_config.h"
    if dst.exists():
        rep.skip(step, "Core/Inc/hardware_config.h 已存在")
        return
    plan.create(dst, (TEMPLATES / "hardware_config.h").read_text(encoding="utf-8"))
    rep.ok(step, "复制到 Core/Inc/（全部默认值，按需取消注释）")


# ==================== 7. 根 CMakeLists.txt ====================


def plan_cmake(host, plan, rep, app_dir):
    step = "CMakeLists.txt"
    tf = TextFile(host / "CMakeLists.txt")
    lib_rel = Path(os.path.relpath(LIB_ROOT, host)).as_posix()
    app_rel = Path(os.path.relpath(host / app_dir, host)).as_posix()
    if re.search(r"add_subdirectory\(\s*" + re.escape(lib_rel) + r"\b", tf.text):
        rep.skip(step, f"已有 add_subdirectory({lib_rel})")
        return
    block = (
        f"\n# stm32_bsp：必须在 add_subdirectory(cmake/stm32cubemx) 之后接入（见 {lib_rel}/README.md 第 1 节）\n"
        f"add_subdirectory({lib_rel})\n"
        "target_link_libraries(${CMAKE_PROJECT_NAME} stm32_bsp)\n"
        f"target_sources(${{CMAKE_PROJECT_NAME}} PRIVATE {app_rel}/board_cfg.cpp {app_rel}/app_main.cpp)\n"
        f"target_include_directories(${{CMAKE_PROJECT_NAME}} PRIVATE {app_rel})\n"
        "target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)  # BspUart::printf 打印浮点需要（nano.specs）\n"
    )
    tf.text = tf.text.rstrip("\n") + "\n" + block
    plan.edit(tf)
    rep.ok(step, f"末尾接入 {lib_rel} 与 {app_rel}/ 的源文件")


# ==================== 8. clangd ====================


def preset_binary_dir(host, preset):
    """按 CMakePresets.json 解析 binaryDir（含 inherits），换成 VS Code 的 ${workspaceFolder}"""
    fallback = f"${{workspaceFolder}}/build/{preset}"
    try:
        presets = json.loads((host / "CMakePresets.json").read_text(encoding="utf-8"))["configurePresets"]
    except (OSError, ValueError, KeyError):
        return fallback
    by_name = {p.get("name"): p for p in presets}
    p = by_name.get(preset)
    while p is not None and "binaryDir" not in p:
        parent = p.get("inherits")
        p = by_name.get(parent[0] if isinstance(parent, list) else parent)
    if p is None:
        return fallback
    return p["binaryDir"].replace("${sourceDir}", "${workspaceFolder}").replace("${presetName}", preset)


def plan_vscode(host, plan, rep, preset):
    step = "clangd"
    gcc = shutil.which("arm-none-eabi-gcc")
    if gcc:
        # 不解析符号链接：CMake 写进编译数据库的是 PATH 里的路径，query-driver 要与之匹配
        driver = str(Path(gcc).parent / "arm-none-eabi-*")
    else:
        driver = "**/arm-none-eabi-*"
        rep.warn(step, "PATH 里没有 arm-none-eabi-gcc，query-driver 用通配 **/arm-none-eabi-*，可改成工具链实际路径")
    args = [f"--compile-commands-dir={preset_binary_dir(host, preset)}", f"--query-driver={driver}"]

    path = host / ".vscode/settings.json"
    if not path.exists():
        db_dir = preset_binary_dir(host, preset).replace("${workspaceFolder}/", "")
        lib_rel = Path(os.path.relpath(LIB_ROOT, host)).as_posix()
        text = (
            "{\n"
            f"    // CMake 预设把编译数据库放在 {db_dir}，clangd 默认只找 build/，需显式指定（先 cmake --preset {preset} 一次）。\n"
            f"    // 命令行参数优先于 {lib_rel}/.clangd 里指向示例工程的 CompilationDatabase，库代码也用本工程的数据库。\n"
            '    "clangd.arguments": [\n'
            + ",\n".join(f"        {json.dumps(a)}" for a in args)
            + "\n    ],\n"
            '    "cmake.useCMakePresets": "always"\n'
            "}\n"
        )
        plan.create(path, text)
        rep.ok(step, ".vscode/settings.json：clangd 编译数据库与 query-driver")
        return
    tf = TextFile(path)
    if "--compile-commands-dir" in tf.text:
        rep.skip(step, ".vscode/settings.json 已指定 --compile-commands-dir")
        return
    try:
        settings = json.loads(tf.text)
    except ValueError:
        rep.warn(step, f".vscode/settings.json 不是纯 JSON（有注释？），请手动在 \"clangd.arguments\" 里加入 {json.dumps(args, ensure_ascii=False)}")
        return
    settings.setdefault("clangd.arguments", [])
    settings["clangd.arguments"] = [a for a in settings["clangd.arguments"] if not a.startswith("--query-driver")] + args
    tf.text = json.dumps(settings, indent=4, ensure_ascii=False) + "\n"
    plan.edit(tf)
    rep.ok(step, ".vscode/settings.json 加入 clangd 编译数据库与 query-driver")


# ==================== 入口 ====================


def default_host():
    out = subprocess.run(
        ["git", "-C", str(LIB_ROOT), "rev-parse", "--show-superproject-working-tree"],
        capture_output=True,
        text=True,
    ).stdout.strip()
    return Path(out) if out else Path.cwd()


def main():
    ap = argparse.ArgumentParser(description="把 stm32_bsp 接入 STM32H7 CubeMX（CMake）宿主工程（README 第 1 节）")
    ap.add_argument("--host", type=Path, help="宿主工程根目录（默认：本库作为 submodule 所在的工程，否则当前目录）")
    ap.add_argument("--dry-run", action="store_true", help="只检查并打印将要做的修改，不写文件")
    ap.add_argument("--app-dir", default="app", help="应用代码目录，相对宿主根（默认 app）")
    ap.add_argument("--linker-script", help="链接脚本，相对宿主根（默认从 cmake/*.cmake 的 -T 参数找）")
    ap.add_argument("--ram-region", help="放 .dma_buffer / .noinit 的内存区（默认按地址找 0x24000000 起的 AXI SRAM，不论叫 RAM_D1 还是 RAM）")
    ap.add_argument("--preset", default="Debug", help="clangd 使用哪个 CMake 预设的编译数据库（默认 Debug）")
    ap.add_argument("--no-hardware-config", action="store_true", help="不复制 hardware_config.h")
    ap.add_argument("--no-vscode", action="store_true", help="不改 .vscode/settings.json")
    args = ap.parse_args()

    host = (args.host or default_host()).resolve()
    if host == LIB_ROOT or not (host / "CMakeLists.txt").is_file():
        sys.exit(f"{host} 不是宿主工程根目录：在宿主工程根目录运行，或用 --host 指定")
    print(f"宿主工程：{host}\n库：{LIB_ROOT}\n")

    rep = Report()
    plan = Plan()
    if check_prerequisites(host, rep):
        plan_linker(host, plan, rep, args.linker_script, args.ram_region)
        plan_freertos_config(host, plan, rep)
        plan_freertos_c(host, plan, rep)
        plan_app(host, plan, rep, args.app_dir)
        if not args.no_hardware_config:
            plan_hardware_config(host, plan, rep)
        plan_cmake(host, plan, rep, args.app_dir)
        if not args.no_vscode:
            plan_vscode(host, plan, rep, args.preset)
    rep.print()

    if rep.has_error:
        print("\n有错误，未修改任何文件。")
        return 1
    if plan.empty():
        print("\n已全部接入，无需修改。")
        return 0
    if args.dry_run:
        print("\n将要做的修改（--dry-run，未写入）：\n")
        plan.show(host)
        return 0
    plan.apply()
    print(
        "\n已写入。下一步：\n"
        f"  1. 按上面的 [注意] 核对 {args.app_dir}/board_cfg.*（句柄、波特率、PWM 参数）\n"
        f"  2. cmake --preset {args.preset} && cmake --build --preset {args.preset}\n"
        "  3. VS Code 里 clangd: Restart language server"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
