#!/usr/bin/env bash
set -e

[ "$#" -eq 1 ] || { echo "user@host argument required" >&2; exit 1; }
HOST=$1

cd ${BASH_SOURCE%/*}
ROOT_DIR=`pwd`/..

rsync -ac -L --partial --progress ${ROOT_DIR}/build_ros2_armhf/install/ ${HOST}:ros2
rsync -ac -L --partial --progress ${ROOT_DIR}/install_armhf/ ${HOST}:ap
