cmake_minimum_required(VERSION 3.11)

project(nxt_com_protocol CXX)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
