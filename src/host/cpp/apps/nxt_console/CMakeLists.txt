cmake_minimum_required(VERSION 3.11)

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

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
