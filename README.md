# CUBOT Code Rebuild：

## 引脚冲突（工程中使用的方案）

1. SPI3 与 串口3 串口5（SPI3的三个引脚）。串口3 与 RS485-3 只能保留一个

- 方案1： U3和U5（舍弃掉SPI3和RS485-3）
- ~~方案2： SPI3和RS485-3（U3和U5）~~

2. IIC1和串口4,引脚复用：选择串口4

## todo

- [x] USB-HID
- [x] QSPI-Flash
- [ ] Cortex‑M7 内核相关
  - [ ] 分支预测
  - [ ] MPU 内存保护单元
  - [ ] Cache 缓存
- [ ] 看门狗、DWT 驱动
- [x] 多种电机驱动开发（DJI DM）
- [ ] 舵机、BMI088、及国产陀螺仪驱动
- [ ] ~~串口协议裁判系统~~、遥控器、~~云台、功率控制、发射机构、超电、底盘控制~~等等

---

- BSP 层的 CAN 接收与 UART 收发已于 2026-10-16 修复并实机验证（见下节「运行期保护与故障现场」）；其余 BSP 未重新实测
- device层中的代码很久没测了，需要测试（`dm_motor.cpp` 目前只有声明、没有实现，见文末）
- app层仍旧在规划，各种test还没写，以及具体的代码应该实现什么，还在规划
- protocol_uart还是很难绷，目前没有代码在用，框架没问题，但是实际没有在用的
- menu太敷衍了，需要移植一个好看的ui出来，我实现的这个非常敷衍而且简单，包括触发使用的方式和处理
- SPI和IIC的回调问题，目前使用的都是阻塞。但是这种通讯模式好像阻塞最适合，因为需要建立连接和等待，例如CS拉高拉低、IIC的处理。

## 运行期保护与故障现场

2026-10-16 修复并实机验证的五项（均在内部回环 / 串口跳线上测过，真实总线上的外部节点、真实电调未测）：

| 项目 | 现在的行为 | 位置 |
| --- | --- | --- |
| CAN 接收 | 先读进 64 字节缓冲；DLC > 8 的帧（FD 帧或经典帧 DLC 9~15）丢弃并计入 `diagnostics.rx_len_drop`，不再写穿中断栈 | `User/Bsp/bsp_can.cpp` |
| UART 收发 | 多任务可并发 `send()` / `printf()`（写锁 + printf 锁）；DMA 启动失败的包留在发送缓冲重发；`sys_task` 每 10 ms 对 8 路串口调 `tx_recover()` / `rx_recover()`，修复卡死的发送与停摆的接收（期间关本串口中断，DMA 句柄卡在 ERROR 时复位）；诊断量见 `rx_diag()` / `tx_diag()` | `User/Bsp/bsp_uart.*`、`User/App/task/sys_task.cpp` |
| 测试开关 | `User/App/app_test.hpp` 中所有测试默认 0，默认固件不驱动任何执行器 | `User/App/app_test.hpp` |
| CAN 指令有效期 | 发送槽位超过 100 ms 没被 `FillData()` / `filldata()` 刷新就清零（只清该槽位），控制任务卡死后电机停转；诊断计数 `expiredCount` | `User/Service/can_tx_node.*`、`User/App/task/can_tx_task.cpp` |
| 初始化失败与故障现场 | `bsp_init()` / `device_init()` 逐条 `configASSERT`；`SysCompleteInit()` 有失败记录时由 `all_init()` 断言停机；`SysFlagWaitRunning()` 等到失败位就退出任务；断言 / 栈溢出 / 分配失败都写进 `fault_record` 再停机（接调试器时 BKPT） | `User/Service/fault_record.*`、`User/Service/status.*` |

UART 接口约定：`send(data, size, timeout_ms, written)`、`receive(buf, size, timeout_ms, received)`，超时单位是毫秒，`portMAX_DELAY` 表示一直等；`Config` 第三个参数是波特率（只给发送侧判卡死用，必须与 CubeMX 一致）。已知限制：7 路 TX DMA 开着 DMA FIFO，卡死恢复时可能丢 DMA FIFO 里预读的字节（约 8~16 字节/次）。

查看故障现场：`fault_record` 放在 RAM_D1 的 `.noinit` 段，复位（看门狗 / 复位键 / 调试器）后仍保留，掉电后靠 `magic` 判断是否有效。用 GDB 挂接后 `print fault_record`，可看到种类（ASSERT / STACK_OVERFLOW / MALLOC_FAILED）、累计次数、文件名与行号、tick、IPSR、任务名。

## 具体文件树

```txt
root              
│                   │.clang-format         代码格式化设置 可以按照自己风格改
│                   │.clangd               clangd 语言服务器配置
│                   │.gitignore            git 忽略文件
│                   │CMakeLists.txt        主构建脚本
│                   │CMakePresets.json     CMake 预设配置
│                   │DM_MC02.ioc           STM32CubeMX 工程文件
│                   │README.md             本文件
│                   │startup_stm32h723xx.s 启动文件
│                   │STM32H723xG_flash.ld  链接脚本（构建实际使用；含 XIP、.dma_buffer、.noinit、USB 邮箱段；CubeMX 重新生成会覆盖它，生成后要还原）
│                   |---
├─.vscode           |`tasks`是终端执行的任务
|                   |如果想使用烧录和调试功能，需要工程名字和文件夹名字一样才可以（elf/hex文件名字和主文件夹名字相同）
├─build             |---
│  └─Debug          |编译出来的临时文件
├─cmake             |---
│  └─stm32cubemx    |STM32的库以及cmake链接方式
├─Core              |---
│  ├─Inc            |STM32的HAL层头文件
│  └─Src            |STM32的HAL层源文件
├─docs              |文档与图片
│  ├─编码规范.md
│  ├─分层规划图.png
│  └─CtrBoard-H7管脚标注图.pdf
├─Drivers           |STM32的Driver层文件
├─Flash             |个人写的烧录相关 工程名字和文件夹名字一样才可以烧录 写好了DAP和JLink 以及win和linux的烧录脚本 ozone
├─Middlewares       |官方生成的中间件库，FreeRTOS，USB
├─QSPI_Flash        |板载 W25Q64JV（OCTOSPI2）驱动、XIP 与外部加载器，见 QSPI_Flash/README.md
├─tinyusb-0.20.0    |TinyUSB库
└─User              |---
    ├─CMakeLists.txt  |User库一键导入配置
    ├─Bsp             |板载支持驱动
    │  ├─bsp_buzzer.cpp / hpp
    │  ├─bsp_can.cpp / hpp        CAN 收发、DLC 过滤、诊断
    │  ├─bsp_can_recovery.cpp     Bus-Off 恢复（sys_task 调用）
    │  ├─bsp_cfg.cpp / hpp        全局实例与 bsp_init()（逐条断言）
    │  ├─bsp_gpio.hpp
    │  ├─bsp_key.cpp / hpp
    │  ├─bsp_uart.cpp / hpp       IDLE+DMA 收、多任务安全发、收发恢复
    │  ├─bsp_usb.cpp / hpp
    │  └─tusb_config.h
    ├─Device          |设备层
    │  ├─device_cfg.cpp / hpp     全局实例与 device_init()（逐条断言）
    │  ├─device_emmv5.cpp / hpp   张大头 Emm_V5（未实例化）
    │  ├─dji_motor.cpp / hpp      DJI 电机（M3508 / M2006 / GM6020）
    │  ├─dm_imu.cpp / hpp         达妙 IMU（未接入接收）
    │  ├─dm_motor.cpp / hpp       达妙电机（只有声明，没有实现）
    │  ├─lcd.cpp / hpp
    │  └─lcd_font.hpp
    ├─Module          |模块层（ 多个设备组合 ）
    │  └─menu.cpp / hpp
    ├─Protocol        |协议层
    │  ├─data_pack.cpp / hpp
    │  ├─protocol_cfg.cpp / hpp
    │  └─protocol_uart.cpp / hpp
    ├─Service         |服务层
    │  ├─can_rx_node.cpp / hpp    CAN 接收节点（按 ID 分发）
    │  ├─can_tx_node.cpp / hpp    CAN 发送节点（槽位聚合、100 ms 指令有效期）
    │  ├─fault_record.cpp / hpp   故障现场记录（.noinit，跨复位保留）
    │  ├─motor_definition.hpp
    │  ├─online_check.cpp / hpp
    │  └─status.cpp / hpp         统一状态码与系统事件标志
    ├─Algorithm       |算法层
    │  ├─pid.cpp
    │  └─pid.hpp
    └─App             |应用层 / c cpp混编接口层
        ├─api_main.cpp / h
        ├─app_message.cpp / hpp
        ├─app_task.hpp
        ├─app_test.hpp            测试开关（默认全部为 0）
        ├─task/       |业务任务（除默认任务外）
        │  ├─can_rx_task.cpp / can_tx_task.cpp
        │  ├─msg_task.cpp
        │  ├─sys_task.cpp         1 kHz：CAN Bus-Off 恢复、每 10 ms 串口补救、Online 计时
        │  └─task_menu.cpp
        └─test/       |测试任务（can / motor / online_check / qspi_flash / uart / usb）

```
我认为：ST官方生成的属于BSP的一部分 以及 Service的一部分

freertos的硬件支持、usb的硬件支持，外设封装好的支持等等，本质上都在硬件抽象（hal）。但是板载支持包（bsp）不仅要硬件抽象，是要完全不考虑硬件，只需要简易的使用代码就可以操控整个板子的外设，不需要考虑板子上的任意情况。这是bsp要做的，也是最麻烦的。

其他的device等等很好理解，只需要在写好的bsp层的基础上，对要做处理的设备进行处理即可

![alt text](分层规划图.png)

## 如何开发

### 编译、烧写和调试

本工程已经配置好 CMake Presets、VS Code Tasks 和 Cortex-Debug。以下 Linux 链路于 **2026-09-01** 在本仓库和实机 STM32H723 上验证通过：

| 能力 | 状态 | 已验证的链路 |
| --- | --- | --- |
| 编译 | 已实测 | CMake 3.22 + Ninja 1.10 + Arm GNU Toolchain 15.3，成功生成 `build/Debug/DM_MC02.elf` |
| 烧写 | 已实测 | CMSIS-DAPv2 + OpenOCD，经 SWD 完成 program、verify 和 reset，校验结果为 `Verified OK` |
| 调试 | 已实测 | OpenOCD GDB Server + `arm-none-eabi-gdb`，成功识别 Cortex-M7、连接目标、复位暂停并读取寄存器 |
| J-Link | 配置存在，当前未实测 | 烧写脚本使用 OpenOCD 的 `jlink` 适配器；当前没有 J-Link 探针，不能据此声称实机链路可用 |
| Windows 脚本 | 配置存在，当前未实测 | `Flash/*.bat` 未在 Windows 环境验证 |

#### 环境要求

- `cmake`、`ninja`、`arm-none-eabi-gcc` 和 `arm-none-eabi-gdb` 在 `PATH` 中。
- CMSIS-DAP 链路需要 `openocd`，并将探针通过 SWD 接到目标板。
- VS Code 图形化调试需要 Cortex-Debug；代码索引建议安装 clangd，任务按钮为可选插件。
- J-Link 烧写需要 OpenOCD 包含 `jlink`/libjaylink 适配器；Ozone 或 J-Link GDB Server 调试仍需安装 SEGGER J-Link Software。
- 当前脚本和 `.vscode/launch.json` 用工作区目录名寻找 ELF，因此工作区目录名、CMake 工程名和输出文件名应保持为 `DM_MC02`。若重命名工程，需要同步修改这些配置。

#### 命令行使用（Linux）

首次配置并编译：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

后续仅编译：

```bash
cmake --build build/Debug
```

使用 CMSIS-DAP 烧写（脚本会分别烧写内部 HEX 和外置 USB XIP BIN，并执行校验和复位）：

```bash
./Flash/OpenOCD_flash.sh
```

注意：第二段需要先让固件跑起来、由固件初始化 OCTOSPI2 后再写外置 Flash。开机就停机的固件（例如故意让 `init()` 失败的测试固件）第二段会失败，并可能把调试口卡死（OpenOCD 无法 examine 内核），需要给板子重新上电；这类固件只烧内部 Flash 即可。

使用 J-Link 探针烧写（经 OpenOCD `jlink` 适配器，当前环境未连接 J-Link 探针）：

```bash
./Flash/JLink_flash.sh
```

#### VS Code 使用

- `Terminal: Run Task` 中可运行 `Configure`、`Build`、`Clean_Rebuild`、`Clean` 以及 Linux/Windows 的 OpenOCD、J-Link 烧写任务；默认构建任务 `Build` 可用 `Ctrl+Shift+B` 运行。
- 在“运行和调试”中选择 `cmsis-dap-debug`，即可通过 OpenOCD 启动 Cortex-Debug，会装载 `build/Debug/DM_MC02.elf` 并运行至 `main`。
- `jlink-debug` 配置已经存在，但需安装 SEGGER J-Link 软件后再使用，当前环境未实测。
- Cortex-Debug 可查看变量、寄存器、内存和基本的 FreeRTOS 运行信息；外设寄存器描述文件位于 `Flash/STM32H723.svd`。
- `Flash/linux.jdebug` 是 Ozone 工程示例，其中含创建者机器的绝对路径、探针序列号和 Ozone 版本信息，换机器后必须在 Ozone 中重新选择 ELF 与探针，不能直接视为通用配置。

相关配置文件：`.vscode/tasks.json`、`.vscode/launch.json`、`Flash/daplink.cfg` 和 `Flash/README.md`。

#### CAN1 驱动 C620/M3508 实机验证

2026-09-01 已使用 C620（绿色双闪，ID 为 2）和 M3508 完成实机验证：CAN 波特率为 1 Mbps，控制帧标准 ID 为 `0x200`，ID2 的电流指令放在 `DATA[2:3]`（高字节在前），反馈帧标准 ID 为 `0x202`。本次成功连接使用同面线；控制板和电调均需正确供电，CANH/CANL 线序及终端电阻应按硬件说明检查。

低电流短时测试已移至 `User/App/test/motor/dji_motor_test.cpp`：启用后保持零输出，调试器写 `dji_motor_test_arm=1` 才执行一次双向测试——原始电流 +512 与 -512（约 0.625 A）各 600 ms，之后滑停并检查方向与位移。测试由 `User/App/app_test.hpp` 中的 `APP_TEST_DJI_MOTOR_ENABLED` 控制；默认值为 `0`（2026-10-16 起），默认固件不会在 CAN1 上注册电机、也不发 0x200。启用测试前必须将电机架空或可靠固定；它与 CAN 恢复测试不能同时打开（文件内 `#error`）。

基于 `DjiMotor`/`DjiMotorBus` 新框架的实测结果为：测试任务通过，收到并成功分发 `0x202` 反馈 20330 帧、忽略 0 帧，峰值转速 3532 rpm，CAN 发送错误计数、接收错误计数和 Bus-Off 均为 0；测试结束后内部 `0x200` 帧四个输出槽位均为零。可在 Live Watch 中查看 `dji_motor_task_*` 和 `dji_motor_test_*` 诊断量。

若 `can1_statu` 从 `Status::OK` 变为持续 `Status::FULL`，应先检查错误计数和 Bus-Off；本次故障由 CAN 连接线序不正确导致，自动重发最终占满发送 FIFO，而不是任务未执行或软件发送缓冲区本身太小。

### 因为底层驱动涉及cpp，解决方法：

严格在每一个.c .cpp中写接口转接文档，每个不能调用的东西都进行接口转接。

写接口转接文档，发现在嵌入式中，只需要给`main.c`进行转接，使用`api_main`来当作提取出来的main即可。这样写的中断回调 初始化 while循环都没啥问题，FreeRTOS也是这样，只需要`extern "C"` 就可以了

HAL库也早就写好了cpp调用c的`extern "C"`内容

作为c调用cpp的转接cpp文件的就写`api_xxxx.cpp`和`api_xxx.h` 作为区分 用**`h`**

正常cpp文件为：`xxx_yyyy.cpp` 和 `xxx_yyyy.hpp` 作为区分 用**`hpp`**

`xxx为 app bsp alg`

## 修改的文件

### CMakeList

为了更方便的添加.c .cpp文件 在User文件夹内添加了一个子文件夹的CMakeLists.txt，这些可以实现一键导入User的库

现在只需要在主CMakeLists.txt的最后一行添加：

```bash
##### User #####
add_subdirectory(User)

# Add stm32cubemx to User_lib
target_link_libraries(User_lib PRIVATE stm32cubemx)

# Add TinyUSB include to Userlib
target_include_directories(User_lib PUBLIC
    ${TINYUSB_DIR}/src
)

# Add User_lib to main
target_link_libraries(${CMAKE_PROJECT_NAME}
    User_lib

)


# 结尾部分
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

##### User #####
```

关于tinyusb，在user后面添加此内容

```bash

### Add TinyUSB sources ###
set(TINYUSB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tinyusb-0.20.0)
include(${TINYUSB_DIR}/src/CMakeLists.txt)
tinyusb_target_add(${CMAKE_PROJECT_NAME})

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${TINYUSB_DIR}/src/portable/synopsys/dwc2/dcd_dwc2.c
    ${TINYUSB_DIR}/src/portable/synopsys/dwc2/dwc2_common.c
)

# Add TinyUSB include to main
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${TINYUSB_DIR}/src
)

# Add project symbols (macros) <= tinyusb
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    CFG_TUSB_MCU=OPT_MCU_STM32H7
    CFG_TUSB_OS=OPT_OS_FREERTOS
)

### Add TinyUSB sources ###
```

以及在最后一行，添加此内容，增加了对浮点数打印的支持

```bash
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
```

### .ld文件

为了DMA传输，把需要用到DMA的东西的存储，换到了DTCM之外

```c
__attribute__((section(".dma_buffer"))) 使用这一个缀修饰
```

修改ld文件的内容如下，中文注释之间为添加内容，多余的内容是定位用的

```c
  .fini_array (READONLY) : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

 /* === 用户为dma传输配置的内存地址 === */

  .dma_buffer (NOLOAD) :
  {
    . = ALIGN(32);
    _sdma_buffer = .;
    *(.dma_buffer)
    *(.dma_buffer*)
    . = ALIGN(32);
    _edma_buffer = .;
  } >RAM_D1

  /* === 用户dma相关配置结束 === */

  /* 故障现场记录：NOLOAD 且不在 .bss 范围内，启动代码不清零，复位后仍保留上一次的内容（掉电丢失） */
  .noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.noinit)
    *(.noinit*)
    . = ALIGN(4);
  } >RAM_D1

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
  } >DTCMRAM AT> FLASH

```

### 让clangd 不报头文件未使用的错误（间接使用 clangd识别不出来）

使用： 在include头文件后面添加

`// IWYU pragma: keep`

可以规避 `Included header XXX.h is not used directly (fixes available)` 这个错误

## 技术细节
### Algorithm 算法
### Bsp 驱动
### Device 设备
#### DM_Motor 达妙电机驱动

> ⚠️ 当前状态：`User/Device/dm_motor.cpp` 只有一行 `#include`，下面的接口只有声明、没有实现（实现在提交 `ea9c6ed` 中删除后未重写）；打开 `APP_TEST_DM_MOTOR_ENABLED` 会编译失败。

##### 使用接口
+ **初始化** 
  - 先声明`DmMotor<电机型号> 电机名称(can_item, esc_id, master_id, mode);`
  - 然后调用`.init()`
+ **数据只读接口**
  - `.data()` 获取电机的运动学数据，以弧度值为单位
  - `.feedback()` 达妙电机的原始反馈数据
  - `.online().isOnline()` 电机在线情况检查
  - `.lvbo_data()` 观测器获得的运动学数据，以弧度值为单位。默认无观测器配置
  - `.drive_state()` 查询电机错误码
  - `.enable_state()` 查询电机使能状态
  - `.status()` 查询最近一次的电机状态
+ **功能接口**
  - `.set_mit_target(position, velocity, kp, kd, torque)` 设置MIT模式下的控制目标
  - `.set_position_velocity_target(position, velocity_limit)` 设置位置-速度模式下的控制目标
  - `.set_velocity_target(velocity)` 设置速度模式下的控制目标
  - `.set_position_torque_target(position, velocity_limit, current_limit_ratio)` 设置力位混控模式下的控制目标
  - `.enable()` 使能电机
  - `.disable()` 失能电机
  - `.clear_error()` 清除电机错误位
  - `.save_current_position_as_zero()` 以当前位置保存为位置零点
##### 技术原理
电机对象持有[电机发送管理对象](#motor_tx_manager)`_tx_endpoint`
##### 实现细节
### Module 模块
### Protocol 协议
### Service 服务
#### motor_tx_manager
#### online_check