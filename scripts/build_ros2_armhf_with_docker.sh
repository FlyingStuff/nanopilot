#/usr/bin/env bash
set -e

cd ${BASH_SOURCE%/*}
ROOT_DIR=`pwd`/..

ROS2_DIR=build_ros2_armhf
echo "building ros2 with docker in ${ROS2_DIR}"

cd ${ROOT_DIR}/scripts/build/ros2_build_container
docker build -t ros2-cross-compile .

cd ${ROOT_DIR}
docker run --rm -v `pwd`:`pwd`:cached -w `pwd` ros2-cross-compile ./scripts/build/build_ros2_armhf.sh ${ROS2_DIR}

# make sure colcon doesn't rebuild everything
touch ${ROS2_DIR}/COLCON_IGNORE

cp -a ${ROS2_DIR}/install ros2_armhf
