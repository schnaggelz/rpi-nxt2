cmake_minimum_required(VERSION 3.11)

project(nxt_remote CXX)

find_package(pybind11 REQUIRED)

add_library(${PROJECT_NAME} STATIC
    include/nxt/remote/remote.hpp
    src/remote.cpp
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

set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

pybind11_add_module(${PROJECT_NAME}_py
    src/remote_py.cpp
)

target_link_libraries(${PROJECT_NAME}_py
    PRIVATE
        ${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/
    FILES_MATCHING PATTERN "*.hpp")

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}_py
    LIBRARY DESTINATION lib
)
