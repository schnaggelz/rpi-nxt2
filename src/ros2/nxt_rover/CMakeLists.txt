cmake_minimum_required(VERSION 3.8)
project(nxt_rover)

find_package(ament_cmake REQUIRED)

# Install URDF and launch files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install package.xml for ament index
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
