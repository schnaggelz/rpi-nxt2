cmake_minimum_required(VERSION 3.8)

project(nxt_app_base CXX)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE 
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    INTERFACE
        nxt_bios
)
