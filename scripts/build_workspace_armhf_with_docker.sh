#/usr/bin/env bash
set -e

# assuming you previously built ros2 for armhf under ROOT_DIR/build_ros2_armhf
COLCON_ARGS=$@

cd ${BASH_SOURCE%/*}
ROOT_DIR=`pwd`/..


cd ${ROOT_DIR}/scripts/build/ros2_build_container
docker build -t ros2-cross-compile .

cd ${ROOT_DIR}
docker run --rm -v `pwd`:`pwd`:cached -w `pwd` -e CROSS_COMPILE=arm-linux-gnueabihf- ros2-cross-compile \
    bash -c "source build_ros2_armhf/install/setup.bash && \
    colcon build --build-base build_armhf --install-base install_armhf ${COLCON_ARGS} --cmake-args -DCMAKE_TOOLCHAIN_FILE=`pwd`/scripts/build/armhf_toolchainfile.cmake"
