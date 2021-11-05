cmake_minimum_required(VERSION 3.10)

project(nxt_console CXX)

add_executable(${PROJECT_NAME}
    source/nxt_console.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        nxt_com
        ncurses
        simple_logger
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
