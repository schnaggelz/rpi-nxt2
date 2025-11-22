cmake_minimum_required(VERSION 3.11)

project(nxt_sw CXX C ASM)

set(LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/fw/nxt.lds)

string(CONCAT LINKER_FLAGS
    " -T${LINKER_FILE}"
    " -Wl,-Map,${PROJECT_NAME}.map")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${LINKER_FLAGS}")

if(NOT "${APP_NAME}" STREQUAL "")
    set(APP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apps/${APP_NAME})
else()
    message(FATAL_ERROR "APP_NAME not set, aborting")
endif()

if (EXISTS "${APP_DIR}" AND EXISTS "${APP_DIR}/CMakeLists.txt")
    message("Using custom path for nxt_sw: ${APP_DIR}")
else()
    message(FATAL_ERROR "APP_DIR=${APP_DIR} does not exist, aborting")
endif()

add_subdirectory(fw)

add_subdirectory(libs/app_utils)
add_subdirectory(libs/app_base)

add_subdirectory(${APP_DIR})
