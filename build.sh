#!/bin/bash

# Create build directory
mkdir -p build
cd build

# Configure and build using CMake
cmake -G "Unix Makefiles" ..
make -j$(sysctl -n hw.ncpu) VERBOSE=1

cd ..
