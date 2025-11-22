# Motor Driver Package

## Building the Package

To build the ROS 2 package, follow these steps:

1. Navigate to the root directory of the package:
   ```
   cd /path/to/motor_driver
   ```

2. Ensure that you have sourced your ROS 2 installation:
   ```
   source /opt/ros/<ros-distro>/setup.bash
   ```

3. Build the package using colcon:
   ```
   colcon build
   ```

4. Source the local setup file after building:
   ```
   source install/setup.bash
   ```

5. You can now run the motor driver node using the launch file:
   ```
   ros2 launch motor_driver motor_driver.launch.py
   ```