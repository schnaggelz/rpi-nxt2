cmake_minimum_required(VERSION 3.11)

project(nxt_remote CXX)

find_package(pybind11 REQUIRED)

add_library(${PROJECT_NAME} STATIC
    include/nxt/remote/remote.hpp
    src/remote.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        nxt_com
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
    EXPORT lib_targets
    LIBRARY DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}_py
    EXPORT lib_targets
    LIBRARY DESTINATION lib
)
