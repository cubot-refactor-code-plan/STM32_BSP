# ------------------------------------------------------------------------------
# stm32_bsp：以 git submodule + add_subdirectory 方式接入 STM32H7 CubeMX（CMake）工程的板级库
#
# 宿主工程（CubeMX 生成的根 CMakeLists.txt，##### User ##### 段）：
#
#   add_subdirectory(third_party/stm32_bsp)
#   target_link_libraries(${CMAKE_PROJECT_NAME} stm32_bsp)
#
# 示例工程见 examples/dm_mc02（它用 add_subdirectory(../.. ...) 接入本库）。
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(FATAL_ERROR
        "stm32_bsp 是库，不能单独配置。请在宿主工程里 add_subdirectory() 接入，"
        "或者构建示例：cd examples/dm_mc02 && cmake --preset Debug")
endif()

if(NOT TARGET stm32cubemx)
    message(FATAL_ERROR
        "找不到 CubeMX 生成的 stm32cubemx 目标：请在 add_subdirectory(cmake/stm32cubemx) 之后再接入 stm32_bsp")
endif()

enable_language(C CXX ASM)

# OBJECT 库而不是 STATIC：库里有覆盖 HAL / FreeRTOS 弱符号的强定义（HAL_*Callback、fault_on_*），
# 静态库成员不会因为“覆盖弱符号”被链接器拉进来，会静默失效。
add_library(stm32_bsp OBJECT)

file(GLOB_RECURSE STM32_BSP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/service/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/device/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/*.cpp
)
target_sources(stm32_bsp PRIVATE ${STM32_BSP_SOURCES})

# 库里不包含任何板级头文件：HAL 回调与库任务按句柄查实例登记表（BspCan / BspUartBase）
target_include_directories(stm32_bsp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/service
    ${CMAKE_CURRENT_SOURCE_DIR}/device
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
)

# HAL / CMSIS / FreeRTOS 的头文件与宏都来自 CubeMX 的 stm32cubemx（INTERFACE）目标
target_link_libraries(stm32_bsp PUBLIC stm32cubemx)

# 用户配置 hardware_config.h：库头文件用 __has_include 在包含路径上找它（见 service/hardware_config_defaults.hpp）。
# 必须放在 stm32cubemx 的包含目录里（推荐 Core/Inc/）：库和用户代码都经由 stm32cubemx 取包含路径，
# 两边看到的是同一份配置；只放在用户自己目标的包含目录里，库会看不到，两边取值不一致。
get_target_property(_stm32bsp_mx_inc stm32cubemx INTERFACE_INCLUDE_DIRECTORIES)
set(_stm32bsp_cfg "")
foreach(_stm32bsp_dir IN LISTS _stm32bsp_mx_inc)
    if(NOT _stm32bsp_cfg AND EXISTS "${_stm32bsp_dir}/hardware_config.h")
        set(_stm32bsp_cfg "${_stm32bsp_dir}/hardware_config.h")
    endif()
endforeach()
if(_stm32bsp_cfg)
    message(STATUS "stm32_bsp: 使用配置 ${_stm32bsp_cfg}")
else()
    message(STATUS "stm32_bsp: 没找到 hardware_config.h，全部用默认值（见 service/hardware_config_defaults.hpp，模板 templates/hardware_config.h）")
endif()

# 链接脚本检查：BspUart 实例要进 .dma_buffer、fault_record 要进 .noinit。CubeMX 新建工程的链接脚本
# 没有这两段，缺了链接器只会把它们当孤儿段随便放（可能落进 DMA 访问不到的 DTCM），运行时才静默出错。
# 所以在配置阶段检查，并把链接脚本登记为配置依赖：它一变（手改、换模板、CubeMX 重新生成），下次构建就重新检查。
# 只认 CMAKE_EXE_LINKER_FLAGS 里的 -T（CubeMX 生成的工具链文件就是这样写的），找不到就跳过。
string(REGEX MATCH "-T[ \t]*\"([^\"]+)\"" _stm32bsp_ld_match "${CMAKE_EXE_LINKER_FLAGS}")
set(_stm32bsp_ld "${CMAKE_MATCH_1}")
if(NOT _stm32bsp_ld)
    string(REGEX MATCH "-T[ \t]*([^ \t\"]+)" _stm32bsp_ld_match "${CMAKE_EXE_LINKER_FLAGS}")
    set(_stm32bsp_ld "${CMAKE_MATCH_1}")
endif()
if(_stm32bsp_ld AND EXISTS "${_stm32bsp_ld}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_stm32bsp_ld}")
    file(READ "${_stm32bsp_ld}" _stm32bsp_ld_text)
    set(_stm32bsp_ld_missing "")
    foreach(_stm32bsp_sec dma_buffer noinit)
        if(NOT _stm32bsp_ld_text MATCHES "\\.${_stm32bsp_sec}[ \t]*(\\(NOLOAD\\))?[ \t\r\n]*:")
            list(APPEND _stm32bsp_ld_missing ".${_stm32bsp_sec}")
        endif()
    endforeach()
    if(_stm32bsp_ld_missing)
        list(JOIN _stm32bsp_ld_missing "、" _stm32bsp_ld_missing)
        message(FATAL_ERROR
            "stm32_bsp: 链接脚本 ${_stm32bsp_ld} 缺少 ${_stm32bsp_ld_missing} 段"
            "。请把 ${CMAKE_CURRENT_SOURCE_DIR}/templates/linker_sections.ld 里的段加进去（放在 SECTIONS 里 .data 之前）。")
    endif()
    message(STATUS "stm32_bsp: 链接脚本检查通过（.dma_buffer、.noinit）：${_stm32bsp_ld}")
else()
    message(STATUS "stm32_bsp: CMAKE_EXE_LINKER_FLAGS 里没找到 -T 链接脚本，跳过 .dma_buffer / .noinit 检查")
endif()

set_target_properties(stm32_bsp PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)
