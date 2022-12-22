cmake_minimum_required(VERSION 3.5)

project(rpi-nxt2)

option(NXT_ARM7 "Build for the NXT2.0 target" OFF)
option(LINUX_HOST "Build for the Linux target" OFF)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(shared/cpp/com_protocol)
add_subdirectory(shared/cpp/common_utils)

if (NXT_ARM7)
    message("Cross-building for ARM")
endif()

if (NXT_ARM7)
    add_subdirectory(nxt2)
endif()

if (LINUX_HOST)
    add_subdirectory(host/cpp/libs)
    add_subdirectory(host/cpp/apps)
endif()
