#!/bin/bash

ROOT_DIR=$(git rev-parse --show-toplevel)

if not [ $# -eq 2 ]
  then
    echo "No parameters specified! Need <app_name> and <build_type>"
    exit 1
fi

BUILD_DIR=${ROOT_DIR}/build/nxt

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake -DCMAKE_BUILD_TYPE="$1" -DAPP_NAME="$2" -DBUILD_NXT_ARM=True \
  -DCMAKE_TOOLCHAIN_FILE="${ROOT_DIR}/cmake/toolchains/arm-gcc-toolchain.cmake" "${ROOT_DIR}"
make clean
make -j 16 VERBOSE=1

cd ..
