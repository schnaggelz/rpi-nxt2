cmake_minimum_required(VERSION 3.11)

project(nxt_common_utils CXX)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include/
    FILES_MATCHING PATTERN "*.hpp")


 install(TARGETS ${PROJECT_NAME}
    EXPORT lib_targets
    LIBRARY DESTINATION lib
)
