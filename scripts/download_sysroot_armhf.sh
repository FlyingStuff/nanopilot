#!/usr/bin/env bash
set -e

[ "$#" -eq 1 ] || { echo "user@host argument required" >&2; exit 1; }
HOST=$1

cd ${BASH_SOURCE%/*}
ROOT_DIR=`pwd`/..

mkdir -p ${ROOT_DIR}/sysroot_armhf
touch ${ROOT_DIR}/sysroot_armhf/COLCON_IGNORE

rsync -az --copy-unsafe-links ${HOST}:/usr/ ${ROOT_DIR}/sysroot_armhf/usr || true
rsync -az --copy-unsafe-links ${HOST}:/lib/ ${ROOT_DIR}/sysroot_armhf/lib || true
rsync -az --copy-unsafe-links ${HOST}:/opt/ ${ROOT_DIR}/sysroot_armhf/opt || true

