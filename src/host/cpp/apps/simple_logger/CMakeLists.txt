cmake_minimum_required(VERSION 3.11)

project(simple_logger CXX)

add_library(${PROJECT_NAME}
    include/simple_logger/logger.hpp
    src/logger.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

install(DIRECTORY include/ DESTINATION include/
    FILES_MATCHING PATTERN "*.hpp"
)
