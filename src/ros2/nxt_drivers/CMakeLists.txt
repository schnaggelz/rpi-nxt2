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

add_executable(motor_driver 
  src/motor_driver/motor_driver.cpp
  src/motor_driver/motor_driver_main.cpp
)

target_include_directories(motor_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(motor_driver PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
)

install(TARGETS motor_driver DESTINATION lib/${PROJECT_NAME})

ament_package()
