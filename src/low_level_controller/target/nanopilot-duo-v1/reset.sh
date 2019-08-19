#!/usr/bin/env bash
set -e

[ "$#" -eq 1 ] || { echo "user@host argument required" >&2; exit 1; }
HOST=$1

BOOT0_IO=203
NRST_IO=363

ssh ${HOST} "sudo sh -c \"echo $NRST_IO > /sys/class/gpio/export; echo out > /sys/class/gpio/gpio$NRST_IO/direction; echo 1 > /sys/class/gpio/gpio$NRST_IO/value ; sleep 0.1; echo 0 > /sys/class/gpio/gpio$NRST_IO/value\""

