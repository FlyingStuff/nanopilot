#!/usr/bin/env bash

# from
# https://github.com/ros2-for-arm/ros2/wiki/ROS2-on-arm-architecture

set -e

echo "building ros2 in ${1}"

cd ${BASH_SOURCE%/*}
ROOT_DIR=`pwd`/../../
cd ${ROOT_DIR}

mkdir -p $1/src
cd $1

export CROSS_COMPILE=arm-linux-gnueabihf-
# export CROSS_COMPILE=aarch64-linux-gnu-
cp ${ROOT_DIR}/scripts/build/armhf_toolchainfile.cmake toolchainfile.cmake
# cp ../aarch64_toolchainfile.cmake toolchainfile.cmake

rm -f ros2.repos
wget https://raw.githubusercontent.com/ros2/ros2/release-crystal-20181214/ros2.repos
vcs-import src < ros2.repos

# hack to make tinyxml2 pass the build: https://github.com/ros2/tinyxml2_vendor/pull/5
cd src/ros2/tinyxml2_vendor
git checkout 0.4.0
cd -

# fix bus error https://github.com/ros2/rcl/pull/365
cd src/ros2/rcl
git reset --hard
curl https://patch-diff.githubusercontent.com/raw/ros2/rcl/pull/365.diff | git apply
cd -

touch \
  src/ros-perception/laser_geometry/COLCON_IGNORE \
  src/ros/resource_retriever/COLCON_IGNORE \
  src/ros2/geometry2/COLCON_IGNORE \
  src/ros2/urdf/COLCON_IGNORE \
  src/ros2/demos/COLCON_IGNORE \
  src/ros2/kdl_parser/COLCON_IGNORE \
  src/ros2/ros1_bridge/COLCON_IGNORE \
  src/ros2/rmw_connext/COLCON_IGNORE \
  src/ros2/orocos_kinematics_dynamics/COLCON_IGNORE \
  src/ros2/examples/rclpy/COLCON_IGNORE \
  src/ros2/robot_state_publisher/COLCON_IGNORE \
  src/ros2/rviz/COLCON_IGNORE \
  src/ros2/rcl/rcl/test/COLCON_IGNORE \
  src/ros2/urdfdom/COLCON_IGNORE \
  src/ros2/rclpy/COLCON_IGNORE \
  src/ros2/rosidl_typesupport_opensplice/COLCON_IGNORE \
  src/ros2/system_tests/COLCON_IGNORE \
  src/ros2/rosidl_python/COLCON_IGNORE \
  src/ros2/rmw_opensplice/COLCON_IGNORE \
  src/ros2/rosidl_typesupport_connext/COLCON_IGNORE \
  src/ros2/rcl_interfaces/test_msgs/COLCON_IGNORE \
  src/ros2/examples/COLCON_IGNORE \
  src/ros2/message_filters/COLCON_IGNORE \
  src/ros-visualization/COLCON_IGNORE

colcon build \
  --cmake-force-configure \
  --cmake-args \
    --no-warn-unused-cli \
    -DCMAKE_TOOLCHAIN_FILE=`pwd`/toolchainfile.cmake \
    -DTHIRDPARTY=ON \
    -DBUILD_TESTING:BOOL=OFF \
    -DCMAKE_BUILD_RPATH="`pwd`/build/poco_vendor/poco_external_project_install/lib/;`pwd`/build/libyaml_vendor/libyaml_install/lib/"
