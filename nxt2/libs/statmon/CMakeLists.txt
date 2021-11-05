project(nxt_monitor CXX)

add_library(${PROJECT_NAME} STATIC
    include/monitor/monitor.hpp
    source/monitor.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        nxt_bios
)
