cmake_minimum_required(VERSION 3.8)

project(nxt_com CXX)

find_package(Threads REQUIRED)

add_library(${PROJECT_NAME}
    include/nxt/usb/device.hpp
    source/device.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        nxt_com_protocol
        nxt_common_utils
    PRIVATE
        pthread
        usb-1.0
)
