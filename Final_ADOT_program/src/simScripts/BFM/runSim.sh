#!/bin/bash

# First argument is endTime
# Second argument is timeStep, 
# Next argument is the write interval
# Next argument is the position number
# Next argument is the test number
# Next argument is whether to run on current process (0) run in parallel (1) or run in parallel with Slurm (2)
# Next argument is the number of nodes
# Next argument is the number of processes per node
# Final argument is the location of the openfoam source script

#Loads latest Openfoam version
openfoamSource=$9

#. $openfoamSource

#Enters case
caseNum="Aerodynamics_Simulation_BFM_Test_$5"
cd $caseNum

#Gets total number of processes
totalProcesses=$(($7*$8))

#Sets controls for simulation
sed -i "/endTime /c\endTime         $1;" system/controlDict
sed -i "/deltaT/c\deltaT          $2;" system/controlDict
sed -i "/writeInterval/c\writeInterval   $3;" system/controlDict

#Replaces files in zero directory
rm -r -f 0/*
cp initialValues/* 0/
cp -r constant/polyMesh 0/

#In case of parallel running
if [ $6 -gt 0 ]; then
    #Updates number of processes
    sed -i "/numberOfSubdomains/c\numberOfSubdomains       $totalProcesses;" system/decomposeParDict
    decomposePar -force > decomposeLog 
fi

#Clears postprocessing files
rm -r -f postProcessing/*


if [ $6 -eq 2 ]; then

    # Load the Intel oneAPI environment for the job
    #source /opt/intel/oneapi/setvars.sh

    # Set the PMI library path for Slurm-MPI integration
    #export I_MPI_PMI_LIBRARY=/opt/slurm/lib/libpmi.so

    echo "Running simulation pos=$4 test=$5 in parallel on $totalProcesses processes across $7 nodes"

    #Sets up slurm script
    sed -i "$((2))s/.*/#SBATCH --job-name=ADOT-Meshing_$3/" simParallel.sh
    sed -i "$((3))s/.*/#SBATCH --nodes=$7/" simParallel.sh
    sed -i "$((4))s/.*/#SBATCH --ntasks-per-node=$8/" simParallel.sh
    sed -i "/bashrc/c\. $openfoamSource " meshParallel.sh

    sbatch --wait --wait-all-nodes 1 simParallel.sh

    rm -r -f processor*
elif [ $5 -eq 1 ]; then
    echo "Running simulation pos=$4 test=$5 in parallel on $totalProcesses processes"
    potentialFoam -parallel -writep > potentialLog
    foamRun -solver incompressibleFluid -parallel > simLog
elif [ $5 -eq 0 ]; then
    echo "Running simulation pos=$4 test=$5"
    potentialFoam -writep > potentialLog
    foamRun -solver incompressibleFluid > simLog
fi

#Clean directories
rm -r -f processor*
