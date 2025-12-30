#!/bin/bash
#SBATCH --job-name=ADOT
#SBATCH --nodes=1    
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=4
#SBATCH --partition=compute
#SBATCH --time=01:00:00
#

# SLURM_NODELIST contains the nodes assigned
export OMPI_MCA_pml=ucx
./Main $SLURM_ARRAY_TASK_ID $SLURM_CPUS_ON_NODE $SLURM_NNODES

#https://stackoverflow.com/questions/38905391/how-can-i-run-mpi-job-in-multiple-nodes-multinode-mpi-job-execution