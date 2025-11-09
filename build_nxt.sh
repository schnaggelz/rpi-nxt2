#!/bin/bash

ROOT_DIR=$(git rev-parse --show-toplevel)

if not [ $# -eq 2 ]
  then
    echo "No parameters specified! Need <app_name> and <build_type>"
    exit 1
fi

BUILD_TYPE=$1
APP_NAME=$2
TARGET_NAME=nxt

BUILD_DIR=${ROOT_DIR}/build/${TARGET_NAME}-${APP_NAME}-${BUILD_TYPE,,}

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DAPP_NAME=${APP_NAME} \
  -DCMAKE_TOOLCHAIN_FILE=${ROOT_DIR}/src/cmake/toolchains/arm-gcc-toolchain.cmake ${ROOT_DIR}/src
make clean
make -j 16 VERBOSE=1

cd ..
