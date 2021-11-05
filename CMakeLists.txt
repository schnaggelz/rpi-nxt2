cmake_minimum_required(VERSION 3.5)

project(rpi-nxt2)

if (BUILD_NXT_ARM)
    message("Cross-building for ARM")
endif()

if (BUILD_NXT_ARM)
    add_subdirectory(nxt2)
endif()

if (BUILD_LINUX)
    add_subdirectory(linux)
endif()
