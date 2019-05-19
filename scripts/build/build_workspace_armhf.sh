#!/usr/bin/env bash

# from https://github.com/ros2/cross_compile

set -e

COLCON_ARGS=$@

cd ${BASH_SOURCE%/*}
ROOT_DIR=`pwd`/../..
cd ${ROOT_DIR}
ROOT_DIR=`pwd` # "there must be no ../ in the path!"

source build_ros2_armhf/install/setup.bash

export TARGET_TRIPLE=arm-linux-gnueabihf
export TARGET_ARCH=armhf
# export TARGET_TRIPLE=aarch64-linux-gnu
# export TARGET_ARCH=aarch64

export CC=/usr/bin/$TARGET_TRIPLE-gcc
export CXX=/usr/bin/$TARGET_TRIPLE-g++
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
export SYSROOT=$ROOT_DIR/sysroot
export PYTHON_SOABI=cpython-36m-$TARGET_TRIPLE
# export ROS2_INSTALL_PATH=$ROOT_DIR/$1/install


colcon build \
  --build-base build_armhf \
  --install-base install_armhf \
  ${COLCON_ARGS} \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=$ROOT_DIR/scripts/cross_compile/cmake-toolchains/generic_linux.cmake \
    -DCMAKE_SYSROOT=$ROOT_DIR/sysroot
