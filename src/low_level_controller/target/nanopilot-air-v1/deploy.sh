#!/usr/bin/env bash
set -e

[ "$#" -eq 1 ] || { echo "user@host argument required" >&2; exit 1; }
HOST=$1

cd ${BASH_SOURCE%/*}
ROOT_DIR=`pwd`

scp ${ROOT_DIR}/build/nanopilot-low-level-controller.bin ${HOST}:llc.bin

BOOT0_IO=203
NRST_IO=6
ssh ${HOST} stm32flash -b 115200 -w llc.bin -v -i "${NRST_IO},${BOOT0_IO},-${NRST_IO},:${NRST_IO},-${BOOT0_IO},-${NRST_IO}" /dev/ttyS1

