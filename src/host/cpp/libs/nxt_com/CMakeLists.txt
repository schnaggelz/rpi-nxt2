cmake_minimum_required(VERSION 3.11)

project(nxt_com CXX)

find_package(Threads REQUIRED)

add_library(${PROJECT_NAME}
    include/nxt/usb/device.hpp
    src/device.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        nxt_com_protocol
        nxt_common_utils
    PRIVATE
        pthread
        usb-1.0
)

install(DIRECTORY include/ DESTINATION include/
    FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS ${PROJECT_NAME}
    EXPORT lib_targets
    LIBRARY DESTINATION lib
)
