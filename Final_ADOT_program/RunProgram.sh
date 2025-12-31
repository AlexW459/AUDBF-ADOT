#!/bin/bash
#SBATCH --job-name=ADOT
#SBATCH -N 1
#SBATCH --ntasks-per-node=2
#SBATCH --cpus-per-task=1
#SBATCH --time=01:00:00
#


spack load openfoam


#/opt/slurm/bin/srun -n2 bash -c 'cp -r Aerodynamics_Simulation "Aerodynamics_Simulation_$SLURM_PROCID"'



/opt/slurm/bin/srun -n1 ./Main 1 2 & spack load sdl2

#https://stackoverflow.com/questions/38905391/how-can-i-run-mpi-job-in-multiple-nodes-multinode-mpi-job-execution
