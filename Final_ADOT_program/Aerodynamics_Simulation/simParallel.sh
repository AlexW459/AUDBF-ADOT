#!/bin/bash
#SBATCH --job-name=ADOT-meshing
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=1
#SBATCH --time=01:00:00
#SBATCH --output=slurmMeshingOutput
#SBATCH --error=slurmMeshingErrorLog
. /opt/openfoam13/etc/bashrc

#source /opt/intel/oneapi/setvars.sh
export I_MPI_PMI_LIBRARY=/opt/slurm/lib/libpmi.so


/opt/slurm/bin/srun potentialFoam -writep -parallel > potentialLog
/opt/slurm/bin/srun foamRun -solver incompressibleFluid -parallel > simLog