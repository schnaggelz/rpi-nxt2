cmake_minimum_required(VERSION 3.10)

project(linux_sw CXX)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(simple_logger)
add_subdirectory(nxt_com)
add_subdirectory(nxt_remote)
add_subdirectory(nxt_console)
