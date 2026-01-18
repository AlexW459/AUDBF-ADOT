#!/bin/bash

#First argument is the case number
#Second number is the number of processes

#Enters case
caseNum="Aerodynamics_Simulation_$1"
cd $caseNum

#. /opt/openfoam13/etc/bashrc
#/apps/spack/share/spack/setup-env.sh
#spack load openfoam

#Cleans mesh
surfaceSplitByTopology aircraftMesh/aircraftModelRaw.obj splitPatches/aircraftModelSplit.obj > splitLog

#Only retains largest mesh
largestAircraftFile=$(wc -l *splitPatches/aircraftModelSplit_*.obj | sort -n | tail -n 2 | head -n 1 | awk '{print $2}')
cp $largestAircraftFile aircraftMesh/aircraftModelSplit.obj
rm splitPatches/*

#Repeats for horizontal stabiliser
surfaceSplitByTopology aircraftMesh/horizontalStabiliserRaw.obj splitPatches/horizontalStabiliserSplit.obj > splitLog2
largestStabiliserFile=$(wc -l *splitPatches/horizontalStabiliserSplit_*.obj | sort -n | tail -n 2 | head -n 1 | awk '{print $2}')
cp $largestStabiliserFile aircraftMesh/horizontalStabiliserSplit.obj
rm splitPatches/*

surfaceLambdaMuSmooth aircraftMesh/aircraftModelSplit.obj 0.5 0.5 20 aircraftMesh/aircraftModel.obj > smoothLog
gzip aircraftMesh/aircraftModel.obj

surfaceLambdaMuSmooth aircraftMesh/horizontalStabiliserSplit.obj 0.5 0.5 20 aircraftMesh/horizontalStabiliser.obj > smoothLog2
gzip aircraftMesh/horizontalStabiliser.obj


rm -f constant/geometry/*
cp aircraftMesh/aircraftModel.obj.gz constant/geometry
cp aircraftMesh/horizontalStabiliser.obj.gz constant/geometry


rm -r -f constant/polyMesh
rm -r -f 0/*

rm -r -f constant/extendedFeatureEdgeMesh

rm -f constant/geometry/aircraftModel.eMesh
rm -f constant/geometry/horizontalStabiliser.eMesh

#Updates number of processes
sed -i "/numberOfSubdomains/c\numberOfSubdomains       $2;" system/decomposeParDict


blockMesh > blockLog
surfaceFeatureExtract > surfaceLog

decomposePar -force -constant > decomposeLog


# Load the Intel oneAPI environment for the job
#source /opt/intel/oneapi/setvars.sh

# Set the PMI library path for Slurm-MPI integration
export I_MPI_PMI_LIBRARY=/opt/slurm/lib/libpmi.so

echo "Running snappyHexMesh in parallel on rank $1 on $2 vCPUs"

srun -N 1 -n $2 snappyHexMesh -parallel -overwrite > meshLog
#snappyHexMesh -overwrite > meshLog

echo "Exiting meshing on rank $1"
exit 0

reconstructPar -constant > reconstructLog

createZones > zoneLog

renumberMesh -constant > renumberLog

rm -r processor*
