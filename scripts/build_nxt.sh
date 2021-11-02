#/bin/bash

if [ $# -eq 0 ]
  then
    echo "No application name specified!"
    exit 1
fi

BUILD_DIR=./build

rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR
cd $BUILD_DIR

# -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-gcc-toolchain
cmake -DAPP_NAME=$1 -DCMAKE_BUILD_TYPE=Debug -DCMAKE_MODULE_PATH=../cmake ../nxt2
make clean
make -j 16 # VERBOSE=1

cd ..
