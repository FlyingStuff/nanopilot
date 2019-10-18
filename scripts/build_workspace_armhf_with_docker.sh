#/usr/bin/env bash
set -e

# assuming you previously built ros2 for armhf under ROOT_DIR/ros2_armhf
COLCON_ARGS=$@

cd ${BASH_SOURCE%/*}
ROOT_DIR=`pwd`/..

cd ${ROOT_DIR}/scripts/build/ros2_build_container
docker build -t ros2-cross-compile .

cd ${ROOT_DIR}
docker run --rm -v `pwd`:`pwd`:cached -w `pwd` ros2-cross-compile ./scripts/build/build_workspace_armhf.sh ${COLCON_ARGS}
