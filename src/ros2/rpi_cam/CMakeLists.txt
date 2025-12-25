cmake_minimum_required(VERSION 3.20)
project(rpi_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig REQUIRED)

# find libcamera
pkg_check_modules(libcamera REQUIRED libcamera>=0.1)

# find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(cam_node src/cam_node.cpp)

target_compile_features(cam_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(cam_node 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(cam_node
  PUBLIC
    ${libcamera_INCLUDE_DIRS}
)

target_link_libraries(cam_node 
  rclcpp::rclcpp
  rclcpp_components::component
)

target_link_libraries(cam_node 
  ${libcamera_LINK_LIBRARIES}
  ${sensor_msgs_TARGETS}
)

install(TARGETS cam_node DESTINATION lib/${PROJECT_NAME})

ament_package()
