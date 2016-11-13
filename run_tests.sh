#!/bin/sh

mkdir -p build/tests
cd build/tests
cmake ../..
make
./tests
