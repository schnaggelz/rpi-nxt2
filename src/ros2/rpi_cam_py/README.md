# Pi Camera Node Package

## Build

Navigate to the root directory of the package:

````sh
cd <ros-src-root>
````

Ensure that you have sourced your ROS 2 installation:

````sh
source /opt/ros/<ros-distro>/setup.sh
````

Build the package using colcon:

````sh
colcon build
````

Source the local setup file after building:

````sh
source install/setup.sh
````

You can now run the motor driver node using the launch file:

````sh
ros2 launch rpi_cam_py cam_node
````
