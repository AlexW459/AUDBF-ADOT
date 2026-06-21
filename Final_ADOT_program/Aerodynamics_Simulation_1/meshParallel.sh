#!/bin/bash
#SBATCH --job-name=ADOT-Meshing_1
#SBATCH --nodes=2
#SBATCH --ntasks-per-node=2
#SBATCH --cpus-per-task=1
#SBATCH --time=01:00:00
#SBATCH --output=slurmMeshingOutput
#SBATCH --error=slurmMeshingErrorLog

#source /opt/intel/oneapi/setvars.sh

export I_MPI_PMI_LIBRARY=/opt/slurm/lib/libpmi.so

 . /home/ubuntu/opt/OpenFOAM-13/etc/bashrc \ 

/opt/slurm/bin/srun snappyHexMesh -parallel -overwrite > meshLog
