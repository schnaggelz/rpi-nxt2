#!/bin/bash

ROOT_DIR=$(git rev-parse --show-toplevel)

if not [ $# -eq 2 ]
  then
    echo "No parameters specified! Need <app_name> and <build_type>"
    exit 1
fi

BUILD_DIR=${ROOT_DIR}/build

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-gcc-toolchain
cmake -DCMAKE_BUILD_TYPE="$1" -DAPP_NAME="$2" -DCMAKE_MODULE_PATH="${ROOT_DIR}/cmake" "${ROOT_DIR}/nxt2"
make clean
make -j 16 # VERBOSE=1

cd ..
