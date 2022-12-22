cmake_minimum_required(VERSION 3.8)

project(nxt_wrappers CXX)

add_library(${PROJECT_NAME} STATIC
    include/wrappers/monitor.hpp
    src/monitor.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        nxt_bios
)
