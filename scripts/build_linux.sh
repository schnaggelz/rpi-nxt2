#!/bin/bash

ROOT_DIR=$(git rev-parse --show-toplevel)

if not [ $# -eq 1 ]
  then
    echo "No parameters specified! Need <build_type>"
    exit 1
fi

BUILD_DIR=${ROOT_DIR}/build/linux

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-gcc-toolchain
cmake -DCMAKE_BUILD_TYPE="$0" "${ROOT_DIR}/linux"
make clean
make -j 16 # VERBOSE=1

cd ..