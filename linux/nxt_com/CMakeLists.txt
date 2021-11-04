cmake_minimum_required(VERSION 3.8)

project(nxt_com CXX)

find_package(Threads REQUIRED)

add_library(${PROJECT_NAME}
    include/nxt_usb/usb_device.hpp
    source/usb_device.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        pthread
        usb-1.0
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
