#!/usr/bin/env bash
set -e

[ "$#" -eq 1 ] || { echo "user@host argument required" >&2; exit 1; }
HOST=$1

cd ${BASH_SOURCE%/*}
ROOT_DIR=`pwd`

scp ${ROOT_DIR}/build/nanopilot-low-level-controller.bin ${HOST}:llc.bin
scp ${ROOT_DIR}/init_gpio.sh ${HOST}:

BOOT0_IO=203
RST_IO=363
ssh ${HOST} sudo stm32flash -b 115200 -w llc.bin -v -i "${RST_IO},${BOOT0_IO},-${RST_IO},:${RST_IO},-${BOOT0_IO},-${RST_IO}" /dev/ttyS3

