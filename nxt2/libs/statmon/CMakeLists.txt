project(nxt_statmon CXX)

add_library(${PROJECT_NAME} STATIC
    include/statmon/status_monitor.hpp
    source/status_monitor.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        nxt_bios
)
