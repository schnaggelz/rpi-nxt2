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

I think the following Python packages should be installed (venv `requirements.txt` TODO):

````sh
pip install colcon-argcomplete colcon-bash colcon-cd colcon-cmake colcon-core colcon-defaults colcon-devtools colcon-library-path colcon-metadata colcon-notification colcon-output colcon-package-information colcon-package-selection colcon-parallel-executor colcon-python-setup-py colcon-recursive-crawl colcon-ros colcon-test-result colcon-zsh  
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

On my Raspi I sorted the generated install commands as listed [here](./base_deps_install.sh).

Now build the workspace:

````sh
colcon build --install-base /opt/ros2 --merge-install --cmake-args -DCMAKE_CXX_FLAGS="-Wno-error=null-dereference -Wno-error=restrict -Wno-error=maybe-uninitialized"
````
