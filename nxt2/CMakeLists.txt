cmake_minimum_required(VERSION 3.5)

include(arm-gcc-toolchain)

project(nxt_sw CXX C ASM)

set(LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/fw/nxt.lds)

string(CONCAT LINKER_FLAGS
    " -T${LINKER_FILE}"
    " -Wl,-Map,${PROJECT_NAME}.map")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${LINKER_FLAGS}")

option(APP_NAME "Application to build" "rc")

add_subdirectory(fw)
add_subdirectory(apps/rc)
