#!/bin/bash
#SBATCH --job-name=ADOT
#SBATCH -N 4
#SBATCH --ntasks-per-node=2
#SBATCH --cpus-per-task=1
#SBATCH --time=01:00:00
#



#/opt/slurm/bin/srun -n2 bash -c 'cp -r Aerodynamics_Simulation "Aerodynamics_Simulation_$SLURM_PROCID"'

#source /opt/intel/oneapi/setvars.sh
export I_MPI_PMI_LIBRARY=/opt/slurm/lib/libpmi.so

spack load openfoam
/opt/slurm/bin/srun -N 4 -n2  ./Main 2

#https://stackoverflow.com/questions/38905391/how-can-i-run-mpi-job-in-multiple-nodes-multinode-mpi-job-execution
