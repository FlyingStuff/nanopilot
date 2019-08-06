#!/usr/bin/env bash

# from https://github.com/ros2/cross_compile

set -e

echo "building ros2 in ${1}"

cd ${BASH_SOURCE%/*}
ROOT_DIR=`pwd`/../..
cd ${ROOT_DIR}
ROOT_DIR=`pwd` # "there must be no ../ in the path!"

mkdir -p $1/src
cd $1

cp ${ROOT_DIR}/scripts/cross_compile/cmake-toolchains/generic_linux.cmake toolchainfile.cmake

export TARGET_TRIPLE=arm-linux-gnueabihf
export TARGET_ARCH=armhf
# export TARGET_TRIPLE=aarch64-linux-gnu
# export TARGET_ARCH=aarch64

export CC=/usr/bin/$TARGET_TRIPLE-gcc
export CXX=/usr/bin/$TARGET_TRIPLE-g++
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
export SYSROOT=$ROOT_DIR/sysroot
export PYTHON_SOABI=cpython-36m-$TARGET_TRIPLE
export ROS2_INSTALL_PATH=$ROOT_DIR/$1/install
echo $ROS2_INSTALL_PATH

# Hack to find Poco
## This is temporarily required to find the Poco libraries on the SYSROOT.
## The exported target comming with the pre-build binaries has a hard-coded
## path to "/usr/lib/<arch>/libz.so" and "/usr/lib/<arch>/libpcre.so"
ln -s `pwd`/../sysroot/lib/$TARGET_TRIPLE/libz.so.1 /usr/lib/$TARGET_TRIPLE/libz.so
ln -s `pwd`/../sysroot/lib/$TARGET_TRIPLE/libpcre.so.3 /usr/lib/$TARGET_TRIPLE/libpcre.so

rm -f ros2.repos
# wget https://raw.githubusercontent.com/ros2/ros2/release-crystal-20190408/ros2.repos
wget https://raw.githubusercontent.com/ros2/ros2/release-dashing-20190614/ros2.repos
vcs-import src < ros2.repos

touch \
  src/ros2/rviz/COLCON_IGNORE \
  src/ros/resource_retriever/COLCON_IGNORE \
  src/ros-visualization/COLCON_IGNORE

colcon build --merge-install \
  --cmake-force-configure \
  --cmake-args \
    -DCMAKE_VERBOSE_MAKEFILE=ON \
    -DCMAKE_TOOLCHAIN_FILE=`pwd`/toolchainfile.cmake \
    -DSECURITY=ON \
    -DBUILD_TESTING=OFF
