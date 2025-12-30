#!/bin/bash
#SBATCH --job-name=ADOT
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=2
#SBATCH --cpus-per-task=1
#SBATCH --time=01:00:00
#

# SLURM_NODELIST contains the nodes assigned
#export OMPI_MCA_pml=ucx

lscpu

spack load openfoam
echo $SLURM_ARRAY_TASK_ID
echo $SLURM_NTASKS
/opt/slurm/bin/srun -n1 ./Main $SLURM_ARRAY_TASK_ID 1 $SLURM_NTASKS

#https://stackoverflow.com/questions/38905391/how-can-i-run-mpi-job-in-multiple-nodes-multinode-mpi-job-execution
