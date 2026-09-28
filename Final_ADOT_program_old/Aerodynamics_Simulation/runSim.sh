#!/bin/bash

# First argument is endTime
# Second argument is timeStep, 
# Next argument is the case number
# Next argument is whether to run on current process (0) run in parallel (1) or run in parallel with Slurm (2)
# Next argument is the number of nodes
# Next argument is the number of processes per node
# Final argument is the location of the openfoam source script

#Loads latest Openfoam version
openfoamSource=$7

. $openfoamSource

#Enters case
caseNum="Aerodynamics_Simulation_$3"
cd $caseNum

#Gets total number of processes
totalProcesses=$(($5*$6))

#Sets controls for simulation
sed -i "/endTime/c\endTime         $1;" system/controlDict
sed -i "/deltaT/c\deltaT          $2;" system/controlDict

#Replaces files in zero directory
rm -r -f 0/*
cp initialValues/* 0/
cp -r constant/polyMesh 0/

#In case of parallel running
if [$4 -gt 0] then
    #Updates number of processes
    sed -i "/numberOfSubdomains/c\numberOfSubdomains       $totalProcesses;" system/decomposeParDict
    decomposePar -force > decomposeLog 
fi

#Clears postprocessing files
rm -r -f postProcessing/*


if [$4 -eq 2] then

    # Load the Intel oneAPI environment for the job
    #source /opt/intel/oneapi/setvars.sh

    # Set the PMI library path for Slurm-MPI integration
    #export I_MPI_PMI_LIBRARY=/opt/slurm/lib/libpmi.so

    echo "Running simulation in parallel on rank $3 on $totalProcesses processes across $5 nodes"

    #Sets up slurm script
    sed -i "$((2))s/.*/#SBATCH --job-name=ADOT-Meshing_$3/" simParallel.sh
    sed -i "$((3))s/.*/#SBATCH --nodes=$5/" simParallel.sh
    sed -i "$((4))s/.*/#SBATCH --ntasks-per-node=$6/" simParallel.sh
    sed -i "/bashrc/c\. $openfoamSource " meshParallel.sh

    sbatch --wait --wait-all-nodes 1 simParallel.sh

    rm -r -f processor*
elif [$4 -eq 1] then
    echo "Running simulation in parallel from rank $3 on $totalProcesses processes"
    potentialFoam -parallel -writep > potentialLog
    foamRun -solver incompressibleFluid -parallel > simLog
elif [$4 -eq 0] then
    echo "Running simulation on rank $3"
    potentialFoam -writep > potentialLog
    foamRun -solver incompressibleFluid > simLog
fi

#Clean directories
rm -r -f processor*
