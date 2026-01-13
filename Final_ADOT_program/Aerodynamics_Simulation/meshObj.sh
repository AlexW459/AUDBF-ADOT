#!/bin/bash

#First argument is the case number
#Second number is the number of processes

#Enters case
caseNum="Aerodynamics_Simulation_$1"
cd $caseNum

. /opt/openfoam13/etc/bashrc


#Cleans mesh
surfaceSplitByTopology aircraftMesh/aircraftModelRaw.obj splitPatches/aircraftModelSplit.obj > splitLog

#Only retains largest mesh
largestFile=$(wc -l *splitPatches/aircraftModelSplit_multiplePart_*.obj | sort -n | tail -n 2 | head -n 1 | awk '{print $2}')
cp $largestFile aircraftMesh/aircraftModel.obj

rm splitPatches/*

surfaceLambdaMuSmooth aircraftMesh/aircraftModelRaw.obj aircraftMesh/aircraftModel.obj 0.5 0.5 20 > smoothLog
gzip aircraftMesh/aircraftModel.obj

surfaceLambdaMuSmooth aircraftMesh/horizontalStabiliserRaw.obj aircraftMesh/horizontalStabiliser.obj 0.5 0.5 20 > smoothLog
gzip aircraftMesh/horizontalStabiliser.obj


rm -f constant/geometry/aircraftModel.obj.gz
cp aircraftMesh/aircraftModel.obj.gz constant/geometry

rm -f constant/geometry/horizontalStabiliser.obj.gz
cp aircraftMesh/horizontalStabiliser.obj.gz constant/geometry


rm -r constant/polyMesh
rm -r -f 0/*

rm -r constant/extendedFeatureEdgeMesh

rm -f constant/geometry/aircraftModel.eMesh
rm -f constant/geometry/horizontalStabiliser.eMesh

#Updates number of processes
sed -i "/numberOfSubdomains/c\numberOfSubdomains       $2;" system/decomposeParDict


blockMesh > blockLog
surfaceFeatures > surfaceLog

decomposePar -force -constant > decomposeLog

# Load the Intel oneAPI environment for the job
source /opt/intel/oneapi/setvars.sh

# Set the PMI library path for Slurm-MPI integration
export I_MPI_PMI_LIBRARY=/opt/slurm/lib/libpmi.so

srun -N 1 -n $2 snappyHexMesh -parallel -overwrite > meshLog
#snappyHexMesh -overwrite > meshLog

reconstructPar -constant > reconstructLog

createZones > zoneLog

renumberMesh -constant > renumberLog

rm -r processor*