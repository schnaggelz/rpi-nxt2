cmake_minimum_required(VERSION 3.5)

project(rpi-nxt2)

option(BUILD_NXT_ARM7 "Build for the NXT2.0 target" OFF)
option(BUILD_LINUX "Build for the Linux target" ON)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(shared/com_protocol)
add_subdirectory(shared/common_utils)

if (BUILD_NXT_ARM7)
    message("Cross-building for ARM")
endif()

if (BUILD_NXT_ARM7)
    add_subdirectory(nxt2)
endif()

if (BUILD_LINUX)
    add_subdirectory(linux/libs/cpp)
    add_subdirectory(linux/apps/cpp)
endif()
