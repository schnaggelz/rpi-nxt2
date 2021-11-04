cmake_minimum_required(VERSION 3.8)

project(nxt_rc CXX)

add_executable(${PROJECT_NAME}
    source/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        nxt_fw
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

generate_object(${PROJECT_NAME})
