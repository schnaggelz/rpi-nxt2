cmake_minimum_required(VERSION 3.10)

project(linux_sw CXX)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(libs/simple_logger)
add_subdirectory(libs/nxt_com)
add_subdirectory(libs/nxt_remote)

add_subdirectory(apps/nxt_console)
