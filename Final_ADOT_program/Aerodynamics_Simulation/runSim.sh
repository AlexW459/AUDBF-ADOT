#!/bin/bash

# First argument is endTime
# Second argument is timeStep, 
# Next argument is the case number
# Final argument is the number of processes

#Enters case
caseNum="Aerodynamics_Simulation_$3"
cd $caseNum

#Sets controls for simulation
endTimeLineNum="$(grep -n "endTime" system/controlDict | head -n 1 | cut -d: -f1)"
deltaTLineNum="$(grep -n "deltaT" system/controlDict | head -n 1 | cut -d: -f1)"

sed -i "$((endTimeLineNum))s/.*/endTime         $1;/" system/controlDict
sed -i "$((deltaTLineNum))s/.*/deltaT          $2;/" system/controlDict


#Replaces files in zero directory
rm -r -f 0/*
cp initialValues/* 0/
cp -r constant/polyMesh 0/

#Updates number of processes
sed -i "/numberOfSubdomains/c\numberOfSubdomains       $4;" system/decomposeParDict

decomposePar -force > decomposeLog 

# Load the Intel oneAPI environment for the job
source /opt/intel/oneapi/setvars.sh

# Set the PMI library path for Slurm-MPI integration
export I_MPI_PMI_LIBRARY=/opt/slurm/lib/libpmi.so

srun -N 1 -n $4 potentialFoam -writep -parallel > potentialLog
srun -N 1 -n $4 foamRun -solver incompressibleFluid -parallel > simLog

#potentialFoam -writep > potentialLog
#foamRun -solver incompressibleFluid > simLog

rm -r -f processor*
