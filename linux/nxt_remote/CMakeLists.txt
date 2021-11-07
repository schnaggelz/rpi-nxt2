cmake_minimum_required(VERSION 3.8)

project(nxt_remote CXX)

add_library(${PROJECT_NAME}
    include/nxt/remote/remote.hpp
    source/remote.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        nxt_com
    PRIVATE
        simple_logger
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
