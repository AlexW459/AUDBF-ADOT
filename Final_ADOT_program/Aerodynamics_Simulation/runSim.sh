#!/bin/bash

# First argument is endTime
# Second argument is timeStep, 
# Next argument is the case number
# Next argument is the number of nodes
# Final argument is the number of processes per node

#Loads latest Openfoam version
#. /usr/lib/openfoam/openfoam2512/etc/bashrc
openfoamSource="/opt/openfoam13/etc/bashrc"

. $openfoamSource

#Enters case
caseNum="Aerodynamics_Simulation_$3"
cd $caseNum

#Sets controls for simulation
sed -i "/endTime/c\endTime         $1;" system/controlDict
sed -i "/deltaT/c\deltaT          $2;" system/controlDict

#Replaces files in zero directory
rm -r -f 0/*
cp initialValues/* 0/
cp -r constant/polyMesh 0/

#Gets total number of processes
totalProcesses=$(($4*$5))

#Updates number of processes
sed -i "/numberOfSubdomains/c\numberOfSubdomains       $totalProcesses;" system/decomposeParDict

decomposePar -force > decomposeLog 

# Load the Intel oneAPI environment for the job
#source /opt/intel/oneapi/setvars.sh

# Set the PMI library path for Slurm-MPI integration
export I_MPI_PMI_LIBRARY=/opt/slurm/lib/libpmi.so

#Clears postprocessing files
rm -r -f postProcessing/aeroForces/0/*
rm -r -f postProcessing/tailAeroForces/0/*
rm -r -f postProcessing/tailUpstreamVelocity/0/*


#Sets up slurm script
sed -i "$((2))s/.*/#SBATCH --job-name=ADOT-Meshing_$3/" simParallel.sh
sed -i "$((3))s/.*/#SBATCH --nodes=$4/" simParallel.sh
sed -i "$((4))s/.*/#SBATCH --ntasks-per-node=$5/" simParallel.sh
sed -i "$((9))s/.*/. $openfoamSource/" simParallel.sh

echo "Running simulation in parallel on rank $3 on $totalProcesses processes across $4 nodes"

potentialFoam -writep > potentialLog
foamRun -solver incompressibleFluid > simLog
#sbatch --wait --wait-all-nodes 1 simParallel.sh

rm -r -f processor*
