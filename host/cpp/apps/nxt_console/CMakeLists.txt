cmake_minimum_required(VERSION 3.10)

project(nxt_console CXX)

add_executable(${PROJECT_NAME}
    src/console.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        nxt_remote
        ncurses
        simple_logger
)
