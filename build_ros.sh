#!/bin/bash

SCRIPT_DIR=$(dirname $(readlink -f "$0"))
ROOT_DIR=$(realpath "$SCRIPT_DIR")

if [ $# -ne 1 ]
  then
    echo "No parameters specified! Need <build_type>"
    exit 1
fi

BUILD_TYPE=$1
TARGET_NAME=ros2

BUILD_DIR=$ROOT_DIR/build/${TARGET_NAME}-${BUILD_TYPE,,}
INSTALL_DIR=$ROOT_DIR/install/${TARGET_NAME}-${BUILD_TYPE,,}

rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR
mkdir -p $INSTALL_DIR

export CMAKE_PREFIX_PATH=$ROOT_DIR/install/linux-${BUILD_TYPE,,}:$CMAKE_PREFIX_PATH

colcon build --base-paths $ROOT_DIR/src/ros2/ --build-base $BUILD_DIR --install-base $INSTALL_DIR --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    --packages-select rpi_cam nxt_drivers
