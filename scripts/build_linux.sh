#!/bin/bash

ROOT_DIR=$(git rev-parse --show-toplevel)

if not [ $# -eq 1 ]
  then
    echo "No parameters specified! Need <build_type>"
    exit 1
fi

BUILD_TYPE="${1,,}"
BUILD_DIR=${ROOT_DIR}/build/linux_${BUILD_TYPE}

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-gcc-toolchain
cmake -DCMAKE_BUILD_TYPE=$1 -DBUILD_LINUX=True ${ROOT_DIR}
make clean
make -j 16 VERBOSE=1

cd ..
