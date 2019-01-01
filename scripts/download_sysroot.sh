#!/usr/bin/env bash
set -e

[ "$#" -eq 1 ] || { echo "user@host argument required" >&2; exit 1; }
HOST=$1

cd ${BASH_SOURCE%/*}
ROOT_DIR=`pwd`/..

mkdir -p ${ROOT_DIR}/sysroot
touch ${ROOT_DIR}/sysroot/COLCON_IGNORE

rsync -avz ${HOST}:/usr/ ${ROOT_DIR}/sysroot/usr
rsync -avz ${HOST}:/lib/ ${ROOT_DIR}/sysroot/lib
