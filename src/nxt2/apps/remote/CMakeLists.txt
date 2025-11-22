cmake_minimum_required(VERSION 3.11)

project(nxt_remote CXX)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/remote.hpp
    src/remote.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        nxt_bios
        nxt_app_utils
        nxt_app_base
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

generate_object(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
    DESTINATION bin
)

