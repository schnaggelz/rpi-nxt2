# ROS2 Setup (Debian Bookworm)

# Build from Source

Install build essentials:

````sh
sudo apt install wget curl build-essential cmake colcon git vcstool
````

Install Python libraries required for building:

````sh
sudo apt install python3-rosdep2 python3-rosinstall-generator python3-setuptools
````

I think the following should be installed in a Python virtual environment (TODO):

````sh
sudo apt install python3-colcon-argcomplete python3-colcon-bash python3-colcon-cd python3-colcon-cmake python3-colcon-core python3-colcon-defaults python3-colcon-devtools python3-colcon-library-path python3-colcon-metadata python3-colcon-notification python3-colcon-output python3-colcon-package-information python3-colcon-package-selection python3-colcon-parallel-executor python3-colcon-python-setup-py python3-colcon-recursive-crawl python3-colcon-ros python3-colcon-test-result python3-colcon-zsh  
````

Create a workspace:

````sh
mkdir ~/ros_ws
cd /ros_ws
````

Now generate the repository list:

````sh
rosinstall_generator ros_base --deps --rosdistro kilted > ros2.repos
````

Modify the generated file as needed and import:

````sh
mkdir src

vcs import src < ros2.repos
````

Check for additional packages to install:

````sh
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1"
````

Now build the workspace:

````sh
colcon build --install-base /opt/ros2 --merge-install --cmake-args -DCMAKE_CXX_FLAGS="-Wno-error=null-dereference -Wno-error=restrict -Wno-error=maybe-uninitialized"
````
