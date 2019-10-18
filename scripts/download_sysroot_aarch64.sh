#!/usr/bin/env bash
set -e

[ "$#" -eq 1 ] || { echo "user@host argument required" >&2; exit 1; }
HOST=$1

cd ${BASH_SOURCE%/*}
ROOT_DIR=`pwd`/..

mkdir -p ${ROOT_DIR}/sysroot_aarch64
touch ${ROOT_DIR}/sysroot_aarch64/COLCON_IGNORE

rsync -az --copy-unsafe-links ${HOST}:/usr/ ${ROOT_DIR}/sysroot_aarch64/usr || true
rsync -az --copy-unsafe-links ${HOST}:/lib/ ${ROOT_DIR}/sysroot_aarch64/lib || true
rsync -az --copy-unsafe-links ${HOST}:/opt/ ${ROOT_DIR}/sysroot_aarch64/opt || true

