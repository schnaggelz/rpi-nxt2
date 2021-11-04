cmake_minimum_required(VERSION 3.5)

include(arm-gcc-toolchain)

project(nxt_sw CXX C ASM)

option(APP_NAME "Application to build" "rc")

add_subdirectory(fw)
#add_subdirectory(apps/rc)
