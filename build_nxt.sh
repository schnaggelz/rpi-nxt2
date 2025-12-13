#!/bin/bash

ROOT_DIR=$(git rev-parse --show-toplevel)

if [ $# -ne 2 ]
  then
    echo "No parameters specified! Need <app_name> <build_type>"
    exit 1
fi

APP_NAME=$1
BUILD_TYPE=$2
TARGET_NAME=nxt

BUILD_DIR=${ROOT_DIR}/build/${TARGET_NAME}-${APP_NAME}-${BUILD_TYPE,,}
INSTALL_DIR=$ROOT_DIR/install/${TARGET_NAME}-${BUILD_TYPE,,}

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DAPP_NAME=${APP_NAME} \
  -DCMAKE_TOOLCHAIN_FILE=${ROOT_DIR}/src/cmake/toolchains/arm-gcc-toolchain.cmake \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ${ROOT_DIR}/src
VERBOSE=1 cmake --build $BUILD_DIR
cmake --install $BUILD_DIR --prefix $INSTALL_DIR

cd ..
