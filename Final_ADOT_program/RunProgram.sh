#!/bin/bash
#SBATCH --job-name=ADOT
#SBATCH -N 2
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=1
#SBATCH --time=01:00:00

source /opt/intel/oneapi/setvars.sh

export I_MPI_PMI_LIBRARY=/opt/slurm/lib/libpmi.so

. $HOME/opt/OpenFOAM-13/etc/bashrc

/opt/slurm/bin/srun -n 2 ./Main 2 2

#https://stackoverflow.com/questions/38905391/how-can-i-run-mpi-job-in-multiple-nodes-multinode-mpi-job-execution


