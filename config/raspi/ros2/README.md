# ROS2 Setup

## Debian Bookworm (From Source)

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

## Ubuntu 24 (e.g. Distrobox)

Install/fix locales (optional  ), e.g.:

````sh
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8 de_DE de_DE.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
````

Setup package repos:

````sh
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
````

Install ROS2, e.g.:

````sh
sudo apt install ros-kilted-desktop
````
