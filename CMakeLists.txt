# ------------------------------------------------------------------------------
# cubot_bsp：以 git submodule + add_subdirectory 方式接入 STM32H7 CubeMX（CMake）工程的板级库
#
# 宿主工程（CubeMX 生成的根 CMakeLists.txt，##### User ##### 段）：
#
#   add_subdirectory(third_party/cubot_bsp)
#   target_link_libraries(${CMAKE_PROJECT_NAME} cubot_bsp)
#
# 示例工程见 examples/dm_mc02（它用 add_subdirectory(../.. ...) 接入本库）。
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(FATAL_ERROR
        "cubot_bsp 是库，不能单独配置。请在宿主工程里 add_subdirectory() 接入，"
        "或者构建示例：cd examples/dm_mc02 && cmake --preset Debug")
endif()

if(NOT TARGET stm32cubemx)
    message(FATAL_ERROR
        "找不到 CubeMX 生成的 stm32cubemx 目标：请在 add_subdirectory(cmake/stm32cubemx) 之后再接入 cubot_bsp")
endif()

enable_language(C CXX ASM)

# OBJECT 库而不是 STATIC：库里有覆盖 HAL / FreeRTOS 弱符号的强定义（HAL_*Callback、fault_on_*），
# 静态库成员不会因为“覆盖弱符号”被链接器拉进来，会静默失效。
add_library(cubot_bsp OBJECT)

file(GLOB_RECURSE CUBOT_BSP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/service/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/device/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/*.cpp
)
target_sources(cubot_bsp PRIVATE ${CUBOT_BSP_SOURCES})

# 库里不包含任何板级头文件：HAL 回调与库任务按句柄查实例登记表（BspCan / BspUartBase）
target_include_directories(cubot_bsp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/service
    ${CMAKE_CURRENT_SOURCE_DIR}/device
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
)

# HAL / CMSIS / FreeRTOS 的头文件与宏都来自 CubeMX 的 stm32cubemx（INTERFACE）目标
target_link_libraries(cubot_bsp PUBLIC stm32cubemx)

# 用户配置 cubot_config.h：库头文件用 __has_include 在包含路径上找它（见 service/cubot_defaults.hpp）。
# 必须放在 stm32cubemx 的包含目录里（推荐 Core/Inc/）：库和用户代码都经由 stm32cubemx 取包含路径，
# 两边看到的是同一份配置；只放在用户自己目标的包含目录里，库会看不到，两边取值不一致。
get_target_property(_cubot_mx_inc stm32cubemx INTERFACE_INCLUDE_DIRECTORIES)
set(_cubot_cfg "")
foreach(_cubot_dir IN LISTS _cubot_mx_inc)
    if(NOT _cubot_cfg AND EXISTS "${_cubot_dir}/cubot_config.h")
        set(_cubot_cfg "${_cubot_dir}/cubot_config.h")
    endif()
endforeach()
if(_cubot_cfg)
    message(STATUS "cubot_bsp: 使用配置 ${_cubot_cfg}")
else()
    message(STATUS "cubot_bsp: 没找到 cubot_config.h，全部用默认值（见 service/cubot_defaults.hpp，模板 templates/cubot_config.h）")
endif()

set_target_properties(cubot_bsp PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)
