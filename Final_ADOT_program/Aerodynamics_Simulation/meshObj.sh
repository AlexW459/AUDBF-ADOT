#!/bin/bash

#First argument is the case number
#Second number is the number of nodes to run on
#Third argument is the processes per node
#Fourth argument is the location of the OpenFOAM sourcing script

openfoamSource=$4


#Enters case
caseNum="Aerodynamics_Simulation_$1"
cd $caseNum

#Gets total number of processes
totalProcesses=$(($2*$3))

#Loads latest Openfoam version
#. /usr/lib/openfoam/openfoam2512/etc/bashrc
. $openfoamSource

#Cleans mesh
surfaceSplitByTopology aircraftMesh/aircraftModelRaw.obj splitPatches/aircraftModelSplit.obj > splitLog

#Only retains largest mesh
largestAircraftFile=$(wc -l *splitPatches/aircraftModelSplit_*.obj | sort -n | tail -n 2 | head -n 1 | awk '{print $2}')
cp $largestAircraftFile aircraftMesh/aircraftModelSplit.obj
rm splitPatches/*

surfaceLambdaMuSmooth aircraftMesh/aircraftModelSplit.obj aircraftMesh/aircraftModel.obj 0.5 0.5 20 > smoothLog
gzip aircraftMesh/aircraftModel.obj


rm -f constant/triSurface/*
cp aircraftMesh/aircraftModel.obj.gz constant/triSurface


rm -r -f constant/polyMesh/*
rm -r -f 0/*

rm -r -f constant/extendedFeatureEdgeMesh/*

#Updates number of processes
sed -i "/numberOfSubdomains/c\numberOfSubdomains       $totalProcesses;" system/decomposeParDict


blockMesh > blockLog
surfaceFeatures > surfaceLog

decomposePar -force -constant > decomposeLog

# Load the Intel oneAPI environment for the job
#source /opt/intel/oneapi/setvars.sh

# Set the PMI library path for Slurm-MPI integration
export I_MPI_PMI_LIBRARY=/opt/slurm/lib/libpmi.so

echo "Running snappyHexMesh in parallel on rank $1 on $totalProcesses processes across $2 nodes"

#Sets up slurm script
sed -i "$((2))s/.*/#SBATCH --job-name=ADOT-Meshing_$1/" meshParallel.sh
sed -i "$((3))s/.*/#SBATCH --nodes=$2/" meshParallel.sh
sed -i "$((4))s/.*/#SBATCH --ntasks-per-node=$3/" meshParallel.sh
sed -i "$((9))s/.*//" meshParallel.sh

sed -i "/bashrc/c\ . $openfoamSource \\" meshParallel.sh

#snappyHexMesh -overwrite > meshLog
sbatch --wait --wait-all-nodes 1 meshParallel.sh

reconstructPar -constant > reconstructLog

createZones > zoneLog

createPatch > patchLog

renumberMesh -constant > renumberLog

rm -r -f processor*
