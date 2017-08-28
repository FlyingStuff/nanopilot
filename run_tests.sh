#!/bin/sh

set -e

mkdir -p build/tests
cd build/tests
cmake ../..
make check
