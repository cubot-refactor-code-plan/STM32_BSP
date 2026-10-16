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

set_target_properties(cubot_bsp PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)
