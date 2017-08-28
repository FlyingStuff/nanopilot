#!/bin/sh

set -e

./run_tests.sh
cd target/INS-board-v1
make
