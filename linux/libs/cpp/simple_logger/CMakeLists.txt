cmake_minimum_required(VERSION 3.10)

project(simple_logger CXX)

add_library(${PROJECT_NAME}
    include/simple_logger/logger.hpp
    source/logger.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    include
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
