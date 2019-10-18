#!/usr/bin/env bash

# from https://github.com/ros2/cross_compile

set -e

COLCON_ARGS=$@

cd ${BASH_SOURCE%/*}
ROOT_DIR=`pwd`/../..
cd ${ROOT_DIR}
ROOT_DIR=`pwd` # "there must be no ../ in the path!"

source ros2_aarch64/setup.bash

# export TARGET_TRIPLE=arm-linux-gnueabihf
# export TARGET_ARCH=armhf
export TARGET_TRIPLE=aarch64-linux-gnu
export TARGET_ARCH=aarch64

export CC=/usr/bin/$TARGET_TRIPLE-gcc
export CXX=/usr/bin/$TARGET_TRIPLE-g++
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
export SYSROOT=$ROOT_DIR/sysroot_aarch64
export PYTHON_SOABI=cpython-36m-$TARGET_TRIPLE
export ROS2_INSTALL_PATH=$ROOT_DIR/

#somehow libtinyxml2.so is not found in sysroot
ln -s ${SYSROOT}/usr/lib/aarch64-linux-gnu/libtinyxml2.so /usr/lib/aarch64-linux-gnu/libtinyxml2.so

colcon build \
  --build-base build_aarch64 \
  --install-base install_aarch64 \
  ${COLCON_ARGS} \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=$ROOT_DIR/scripts/cross_compile/cmake-toolchains/generic_linux.cmake \
    -DCMAKE_SYSROOT=$ROOT_DIR/sysroot_aarch64
