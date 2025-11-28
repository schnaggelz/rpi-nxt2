cmake_minimum_required(VERSION 3.20)
project(nxt_drivers)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(nxt_libs REQUIRED)

add_executable(driver_node 
    src/driver_node.cpp
    src/node_main.cpp
)

target_include_directories(driver_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(driver_node PUBLIC
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
    nxt::nxt_remote
)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MotorCommand.msg"
)

install(TARGETS driver_node DESTINATION lib/${PROJECT_NAME})

ament_package()
