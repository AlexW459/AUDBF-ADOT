#!/bin/sh

# Remove old cmake files
#cd sdfibm/src
#cmake -S .. -Wno-dev
#make
#cd ../../


# Compiler flags
CXXFLAGS="-std=c++23 -Wall -march=x86-64 -msse2 -O3 -DPROJECT_ROOT=\\\"$(pwd)\\\""
#-fopt-info-vec-missed -fno-
#-fsanitize=address
COMPILEFLAGS="-lSDL2"

cd src/
make $1 "CXXFLAGS=$CXXFLAGS" "COMPILEFLAGS=$COMPILEFLAGS"
