#!/bin/bash

#First argument is the case number

#Enters case
caseNum="Aerodynamics_Simulation_$1"
cd $caseNum

. /opt/openfoam13/etc/bashrc


#Cleans mesh
surfaceSplitByTopology aircraftMesh/aircraftModelRaw.obj splitPatches/aircraftModelSplit.obj

#Only retains largest mesh
largestFile=$(wc -l *splitPatches/aircraftModelSplit_multiplePart_*.obj | sort -n | tail -n 2 | head -n 1 | awk '{print $2}')
cp $largestFile aircraftMesh/aircraftModel.obj

rm splitPatches/*

surfaceLambdaMuSmooth aircraftMesh/aircraftModelRaw.obj aircraftMesh/aircraftModel.obj 0.5 0.5 20 
gzip aircraftMesh/aircraftModel.obj

surfaceLambdaMuSmooth aircraftMesh/horizontalStabiliserRaw.obj aircraftMesh/horizontalStabiliser.obj 0.5 0.5 20 
gzip aircraftMesh/horizontalStabiliser.obj


rm constant/geometry/aircraftModel.obj.gz
cp aircraftMesh/aircraftModel.obj.gz constant/geometry

rm constant/geometry/horizontalStabiliser.obj.gz
cp aircraftMesh/horizontalStabiliser.obj.gz constant/geometry


rm -r constant/polyMesh
rm -r 0/*

rm -r constant/extendedFeatureEdgeMesh

rm constant/geometry/aircraftModel.eMesh
rm constant/geometry/horizontalStabiliser.eMesh


blockMesh > blockLog
surfaceFeatures > surfaceLog

#rm -r -f processer*

#decomposePar -force

snappyHexMesh -overwrite > meshLog
createZones

renumberMesh -constant

#mpirun -np $1 snappyHexMesh -overwrite > meshLog

#rm -r constant/polyMesh

#reconstructPar -constant -overwrite

