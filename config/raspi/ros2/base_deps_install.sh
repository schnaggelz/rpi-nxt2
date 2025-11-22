#!/bin/bash

# Install C/C++ libraries (absolute minimum for ROS2 base on Raspberry Pi OS)

sudo -H apt-get install -y libspdlog-dev
sudo -H apt-get install -y libconsole-bridge-dev
sudo -H apt-get install -y liblttng-ust-dev
sudo -H apt-get install -y libssl-dev
sudo -H apt-get install -y libgtest-dev

# Install Python packages (absolute minimum for ROS2 base on Raspberry Pi OS)

sudo -H apt-get install -y python3-setuptools
sudo -H apt-get install -y python3-packaging
sudo -H apt-get install -y python3-pytest

# sudo -H apt-get install -y python3-pytest-mock
# sudo -H apt-get install -y python3-pytest-timeout
# sudo -H apt-get install -y libeigen3-dev
# sudo -H apt-get install -y liborocos-kdl-dev
# sudo -H apt-get install -y libyaml-cpp-dev
# sudo -H apt-get install -y python3-pykdl
# sudo -H apt-get install -y cppcheck
# sudo -H apt-get install -y libtinyxml2-dev
# sudo -H apt-get install -y libxml2-utils
# sudo -H apt-get install -y pydocstyle
# sudo -H apt-get install -y python3-importlib-resources
# sudo -H apt-get install -y graphviz
# sudo -H apt-get install -y lttng-tools
# sudo -H apt-get install -y google-mock
# sudo -H apt-get install -y libacl1-dev
# sudo -H apt-get install -y libasio-dev
# sudo -H apt-get install -y cargo
# sudo -H apt-get install -y clang
# sudo -H apt-get install -y python3-pycodestyle
# sudo -H apt-get install -y python3-lark
# sudo -H apt-get install -y clang-format
# sudo -H apt-get install -y pybind11-dev
# sudo -H apt-get install -y rti-connext-dds-7.3.0-ros
# sudo -H apt-get install -y libsqlite3-dev
# sudo -H apt-get install -y python3-flake8
# sudo -H apt-get install -y uncrustify
# sudo -H apt-get install -y libyaml-dev
# sudo -H apt-get install -y nlohmann-json3-dev
# sudo -H apt-get install -y libbullet-dev
# sudo -H apt-get install -y python3-flake8-blind-except
# sudo -H apt-get install -y python3-flake8-builtins
# sudo -H apt-get install -y python3-flake8-class-newline
# sudo -H apt-get install -y python3-flake8-comprehensions
# sudo -H apt-get install -y python3-flake8-deprecated
# sudo -H apt-get install -y python3-flake8-docstrings
# sudo -H apt-get install -y python3-flake8-import-order
# sudo -H apt-get install -y python3-flake8-quotes
# sudo -H apt-get install -y liblz4-dev
# sudo -H apt-get install -y libbenchmark-dev
