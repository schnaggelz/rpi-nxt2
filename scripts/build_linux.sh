#!/bin/bash

SCRIPT_DIR=$(dirname $(readlink -f "$0"))
ROOT_DIR=$(realpath "$SCRIPT_DIR/..")

if [ $# -ne 2 ]
  then
    echo "No parameters specified! Need <target_name> <build_type>"
    exit 1
fi

TARGET_NAME=$1
BUILD_TYPE=$2

BUILD_DIR=${ROOT_DIR}/build/${TARGET_NAME}-linux-${BUILD_TYPE,,}
INSTALL_DIR=${HOME}/.local

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DBUILD_LINUX=True ${ROOT_DIR}
make clean
make -j 16 VERBOSE=1
make install

cd ..
