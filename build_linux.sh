#!/bin/bash

SCRIPT_DIR=$(dirname $(readlink -f "$0"))
ROOT_DIR=$(realpath "$SCRIPT_DIR")

if [ $# -ne 1 ]
  then
    echo "No parameters specified! Need <build_type>"
    exit 1
fi

BUILD_TYPE=$1

BUILD_DIR=$ROOT_DIR/build/linux-${BUILD_TYPE,,}
INSTALL_DIR=$ROOT_DIR/install/linux-${BUILD_TYPE,,}

rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR
cd $BUILD_DIR

cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DBUILD_LINUX=True $ROOT_DIR/src
cmake --build $BUILD_DIR
cmake --install $BUILD_DIR --prefix $INSTALL_DIR

cd ..
