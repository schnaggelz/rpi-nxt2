cmake_minimum_required(VERSION 3.11)

project(nxt_app_utils CXX)

add_library(${PROJECT_NAME} STATIC
    include/utils/status_monitor.hpp
    src/status_monitor.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        nxt_bios
)
