cmake_minimum_required(VERSION 3.5)
project(motor_driver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable for the motor driver node
add_executable(motor_driver_node src/motor_driver_node.cpp)
ament_target_dependencies(motor_driver_node rclcpp std_msgs)

# Add executable for the motor interface
add_executable(motor_interface src/drivers/motor_interface.cpp)
ament_target_dependencies(motor_interface rclcpp std_msgs)

# Install targets
install(TARGETS
  motor_driver_node
  motor_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install message files
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

# Install service files
install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Install test files
install(DIRECTORY test/
  DESTINATION share/${PROJECT_NAME}/test
)

ament_package()