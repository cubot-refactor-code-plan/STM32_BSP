# 烧写调试方式评估：EmberProbe skill vs 命令行（OpenOCD / GDB / CubeMX CLI）

环境：EmberProbe 0.8.6（VS Code 扩展，skill 装在 `.agents/skills/`，经 `.claude/skills` 软链接给 Claude Code）、
OpenOCD 0.12.0+dev（`/usr/local/bin/openocd`，配置 `examples/dm_mc02/Flash/daplink.cfg`）、CMSIS-DAP（Horco，SN 5335231732）、
STM32CubeMX 6.18.1、arm-none-eabi-gdb。本次测试共烧写 17 次（16 次 EmberProbe、1 次 OpenOCD 命令行），CubeMX 生成 2 次。

## 1. 逐项数据

### 烧写

| 方式 | 一次耗时 | 人工确认 | 结果 |
| --- | --- | --- | --- |
| EmberProbe `mcu-flash`（预检 + 执行） | 预检 0.1 s，执行 3.9 s | **每次都要**（只有 once / deny，没有 24 h 授权） | 16/16 成功，program + verify + reset |
| OpenOCD `program … verify reset exit` | 3.4 s | 无（命令行本身没有确认环节） | 1/1 成功 |

EmberProbe 预检会报告 ELF 路径与 SHA-256、目标、探针序列号、OpenOCD 版本与各值来源，执行前给出确认 ID —— 能明确“烧的是哪个文件”，
但测试迭代时每轮都要用户点一次。耗时差别可以忽略。

### 读变量 / 寄存器

| 方式 | 一次耗时 | 是否停核 | 能力与限制 |
| --- | --- | --- | --- |
| EmberProbe `mcu-variables` | 0.2~0.3 s | 不停 | 按 DWARF 解析全局变量、结构体成员、数组元素、**类的私有成员**（如 `bsp_uart7._tx_chunk_len`）、指针 `->` 路径；JSON 输出好脚本化。限制：`char[]` 返回数字数组；文件作用域 `static` 对象解析失败（`COMPOSITE_LAYOUT_MISSING`），且**一批里一个名字失败整批失败**；运行中**只读 ELF 内 RAM，外设寄存器拒读**（`LIVE_ADDRESS_NOT_RAM`）；Release（`-g0`）构建没有 DWARF 读不了结构体；**按磁盘上当前 ELF 解析地址，ELF 与板上固件不一致时不报警**（测试中必须注意先读完再重新编译） |
| OpenOCD `mdw` / `dump_image` | 0.1 s；64 KB 导出 0.5 s | 不停 | 任意地址（外设寄存器、大块 RAM），本次定位 BRR = 0、DMA EN 卡住、导出字节日志都靠它；只有原始字，需要自己用 `nm` 查地址、算偏移 |
| GDB batch `print` | 0.16 s | **连接时停核** | 显示最好（结构体、字符串）；停核会改变时序，不适合观察运行中的问题 |
| EmberProbe `mcu-chip-info` | 约 0.5 s | 短暂占用 | 芯片识别、运行状态（running / halted）方便 |

### CubeMX 修改与生成（mcu-cubemx）

| 次数 | 结果 |
| --- | --- |
| 第 1 次（USART2 / USART3） | 候选生成、预检（95 项变更分类、语法与结构检查）都很好；执行失败 `CUBEMX_LAYOUT_UNSUPPORTED`：工程里的 `.claude/skills` 软链接不受支持 |
| 第 1 次重试（临时删软链接） | 失败 `CUBEMX_GENERATION_FAILED`，failureLine 是 CubeMX 首页广告 / 在线更新面板的 `[ERROR] WebUtils - Updater is busy`；日志里 baseline 代码其实已全部生成、CubeMX 正常退出 —— **失败判定误报**（当时用户恰好开着 CubeMX 界面） |
| 第 2 次（TIM8 优先级，授权 24 h） | 没有开 CubeMX 界面，**同样误报失败** |
| 回退 CubeMX 命令行 | 两次都成功（各 25 s），用的是 EmberProbe 生成并校验过的候选 `.ioc`；第一次命令行卡死 300 s 是因为桌面开着 CubeMX 界面抢锁 |

### 其他 skill

- `mcu-config`：读写工作区配置（切 ELF 路径）正常、即时。
- `mcu-chip-info`：识别 Cortex-M7 / STM32H7x / 0x483 / 1024 KiB，`runtime` 段能区分 running / halted。
- `mcu-debug-control`、`mcu-rtos`、`mcu-peripheral-debug`：都要求暂停的 Cortex-Debug 会话；本次的问题（运行中卡死、运行中寄存器状态）都需要**不停核**观察，没有用上。
- Claude Code 刚启动时 skill 列表里没有 `mcu-flash`（其他 9 个都有），稍后自动出现。

## 2. 关键事件里两种方式的表现

| 事件 | 用了什么 | 说明 |
| --- | --- | --- |
| U04 在 2 Mbps 卡住 | EmberProbe 读驱动私有状态（`_tx_stall_rec_cnt` 11615 次）→ OpenOCD 读 UART7 寄存器发现 BRR = 0 | 前者看软件状态方便，后者看外设不可替代 |
| 921600 下确定性丢帧、4 Mbps 残余丢失 | 固件记字节日志 → OpenOCD `dump_image` 导出 64 KB×2 → Python 比对 | EmberProbe 单次读上限 4096 字节，不适合大块导出 |
| -O2 整机卡死 | EmberProbe 发现 `sys_loop_count` 不动、`fault_record` 无记录、目标 running → OpenOCD 停核读 PC / IPSR / DMA 寄存器、手动写 EN、`objdump` 反汇编 HAL | 根因定位全在命令行完成 |
| 物理插拔测试的人机配合 | EmberProbe 轮询 `bsp_test_prompt` 与结果表 | 不停核、JSON 输出，适合轮询 |

## 3. 结论与今后的分工

| 操作 | 优先用 | 理由 |
| --- | --- | --- |
| 正式烧写（给人看、需要留痕） | EmberProbe `mcu-flash` | 预检报告 ELF 指纹 / 目标 / 探针，确认环节防止烧错文件 |
| 测试迭代中的反复烧写 | OpenOCD 命令行（经用户一次性同意） | 时间相同，但 EmberProbe 每次都要人工确认、没有 24 h 选项，多轮迭代时打断太多 |
| 运行中读变量 / 结构体 / 私有成员、轮询状态 | EmberProbe `mcu-variables` | 不停核、按 DWARF 自动解析、JSON 输出；注意读之前确认磁盘 ELF 与板上一致 |
| 运行中读外设寄存器、导出大块内存、写内存救急 | OpenOCD 命令行 | EmberProbe 运行中不读外设、单次读有上限 |
| 卡死 / HardFault 现场（PC、IPSR、调用关系） | OpenOCD 停核 + `addr2line` / `objdump`；`mcu-fault-analyzer` 用于 HardFault 寄存器解码 | 本次卡死不是 fault，`mcu-fault-analyzer` 不适用 |
| CubeMX 改配置 | EmberProbe 生成候选 + 预检审阅变更，**生成用 CubeMX 命令行** | EmberProbe 的生成在本机两次因 “Updater is busy” 误报失败，且不支持工程内软链接 |

给 EmberProbe 的反馈点：mcu-flash 增加有时限的授权；mcu-variables 支持运行中读外设（或给出只读开关）、`char[]` 渲染成字符串、批量读部分失败不影响其他、检测 ELF 与目标不一致；mcu-cubemx 不把 CubeMX 日志里与生成无关的 `[ERROR]` 判为失败、支持工程内软链接。
