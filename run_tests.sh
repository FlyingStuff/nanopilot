#!/bin/sh

python packager/packager.py

mkdir -p build/tests
cd build/tests
cmake ../..
make
./tests
