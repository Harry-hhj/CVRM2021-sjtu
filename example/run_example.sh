#!/bin/bash
mkdir cmake-build-debug
cd cmake-build-debug
cmake ..
make -j4
./example ../app.py