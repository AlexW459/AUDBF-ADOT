#!/bin/bash

#Cleans mesh
#surfaceCheck aircraftMesh/aircraftModelRaw.obj -splitNonManifold
#mv aircraftModelRaw*.obj splitPatches

#Only retains largest mesh
#largestFile=$(wc -l *splitPatches/aircraftModelRaw*.obj | sort -n | tail -n 2 | head -n 1 | awk '{print $2}')
#cp $largestFile aircraftMesh/aircraftModelSplit.obj
#rm splitPatches/*

cd Aerodynamics_Simulation

surfaceLambdaMuSmooth aircraftMesh/aircraftModelRaw.obj aircraftMesh/aircraftModel.obj 0.5 0.5 20

gzip aircraftMesh/aircraftModel.obj

rm constant/geometry/aircraftModel.obj.gz
cp aircraftMesh/aircraftModel.obj.gz constant/geometry

rm -r 0/*
cp -a initialValues/. 0/

rm -r constant/polyMesh

rm -r constant/extendedFeatureEdgeMesh

rm constant/geometry/aircraftModel.eMesh

blockMesh
surfaceFeatures

#decomposePar -copyZero

#mpirun -np 4 snappyHexMesh -parallel -overwrite
snappyHexMesh -overwrite

#find . -type f -iname "*level*" -exec rm {} \;

#mpirun renumberMesh -constant -overwrite -parallel
renumberMesh -constant -overwrite

#Copies to zero directories
#for dir in processor*; do
#    if [ -d "$dir/constant/polyMesh" ]; then
#        mkdir -p $dir/0
#        cp -r $dir/constant/polyMesh $dir/0/
#    fi
#done

cp -r constant/polyMesh 0/

#mpirun potentialFoam -initialiseUBCs -parallel -writep

potentialFoam -initialiseUBCs -writep

#reconstructPar -withZero

#rm -r  processor*
