cmake_minimum_required(VERSION 3.5)

project(rpi-nxt2)

option(NXT_ARM7 "Build for the NXT2.0 target" OFF)
option(LINUX_HOST "Build for the Linux target" OFF)
option(APP_NAME "Application name to build" "nonexisting")


set(CMAKE_CXX_STANDARD 17 CACHE STRING "")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(shared/cpp/com_protocol)
add_subdirectory(shared/cpp/common_utils)

if (CMAKE_CROSSCOMPILING)
    message("Cross-building for ARM")
endif()

if (CMAKE_CROSSCOMPILING)
    add_subdirectory(nxt2)
else()
    add_subdirectory(host/cpp/libs)
    add_subdirectory(host/cpp/apps)
endif()
