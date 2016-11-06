#!/bin/sh

python3 packager/cvra_packager/packager.py

mkdir -p build/tests
cd build/tests
cmake ../..
make
./tests
