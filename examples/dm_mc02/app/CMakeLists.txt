# DM-MC02 示例的板级用户代码：实例配置（board/）、api_main、业务任务（task/）与实机测试（test/）。
# 新增 .c / .cpp 文件无需改本文件。

add_library(dm_mc02_app OBJECT)

file(GLOB_RECURSE DM_MC02_APP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)
target_sources(dm_mc02_app PRIVATE ${DM_MC02_APP_SOURCES})

# PUBLIC：Core/Src/main.c 与 freertos.c 要包含 api_main.h
target_include_directories(dm_mc02_app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/board
)

target_link_libraries(dm_mc02_app PUBLIC cubot_bsp stm32cubemx)

set_target_properties(dm_mc02_app PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)
