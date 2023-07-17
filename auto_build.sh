#!/bin/bash

# Exit on error
set -e

if [ ! -e "./build" ];then
  mkdir build
  echo "create ./build/"
fi

echo "start cmake build"
cd ./build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4