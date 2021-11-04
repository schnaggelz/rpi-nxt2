cmake_minimum_required(VERSION 3.8)

project(nxt_rc CXX)

find_package(Threads REQUIRED)

add_executable(${PROJECT_NAME}
    source/nxt_rc.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        nxt_com
        ncurses
)
