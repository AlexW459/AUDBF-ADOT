#!/bin/bash

#First argument is the case number
#Second argument is the location of the OpenFOAM sourcing script
#Third argument is whether to run on current process (0) run in parallel (1) or run in parallel with Slurm (2)
#Fourth argument is the number of nodes/processes to run on 
#Fifth argument is the processes per node

openfoamSource=$2

. $openfoamSource

#Enters case
caseNum="Aerodynamics_Simulation_$1"
cd $caseNum

#Gets total number of processes
totalProcesses=$(($3*$4))

#Loads latest Openfoam version
. $openfoamSource

#Cleans mesh
surfaceSplitByTopology aircraftMesh/aircraftModelRaw.obj splitPatches/aircraftModelSplit.obj > splitLog

#Only retains largest mesh
largestAircraftFile=$(wc -l *splitPatches/aircraftModelSplit_*.obj | sort -n | tail -n 2 | head -n 1 | awk '{print $2}')
cp $largestAircraftFile aircraftMesh/aircraftModelSplit.obj
rm splitPatches/aircraftModel*

surfaceLambdaMuSmooth aircraftMesh/aircraftModelSplit.obj aircraftMesh/aircraftModel.obj 0.5 0.5 20 > smoothLog
gzip aircraftMesh/aircraftModel.obj


rm -f constant/geometry/aircraftModel*
cp aircraftMesh/aircraftModel.obj.gz constant/geometry/


rm -r -f constant/polyMesh/*
rm -r -f 0/*

rm -r -f constant/extendedFeatureEdgeMesh/*


blockMesh > blockLog
surfaceFeatures > surfaceLog

#In case of parallel running
if [$3 -gt 0] then
    #Updates number of processes
    sed -i "/numberOfSubdomains/c\numberOfSubdomains       $totalProcesses;" system/decomposeParDict
    decomposePar -force -constant > decomposeLog
fi

if [$3 -eq 2] then

    # Load the Intel oneAPI environment for the job
    #source /opt/intel/oneapi/setvars.sh

    # Set the PMI library path for Slurm-MPI integration
    #export I_MPI_PMI_LIBRARY=/opt/slurm/lib/libpmi.so

    echo "Running snappyHexMesh in parallel on rank $1 on $totalProcesses processes across $4 nodes"

    #Sets up slurm script
    sed -i "$((2))s/.*/#SBATCH --job-name=ADOT-Meshing_$1/" meshParallel.sh
    sed -i "$((3))s/.*/#SBATCH --nodes=$4/" meshParallel.sh
    sed -i "$((4))s/.*/#SBATCH --ntasks-per-node=$5/" meshParallel.sh
    sed -i "/bashrc/c\. $openfoamSource" meshParallel.

    sbatch --wait --wait-all-nodes 1 meshParallel.sh

    reconstructPar -constant > reconstructLog
elif [$3 -eq 1] then
    echo "Running snappyHexMesh in parallel from rank $1 on $totalProcesses processes"
    snappyHexMesh -parallel -overwrite > meshLog
    reconstructPar -constant > reconstructLog
elif [$3 -eq 0] then
    echo "Running snappyHexMesh on rank $1"
    snappyHexMesh -overwrite > meshLog
fi

createZones > zoneLog

createPatch > patchLog

renumberMesh -constant > renumberLog

#Clean directories
rm -r -f processor*
