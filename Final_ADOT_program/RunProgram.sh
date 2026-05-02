#!/bin/bash
#SBATCH --job-name=ADOT
#SBATCH -N 2
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=1
#SBATCH --time=01:00:00


export I_MPI_PMI_LIBRARY=/opt/slurm/lib/libpmi.so

. $HOME/opt/OpenFOAM-13/etc/bashrc

#Include following commands in bash script to manually specify locations of shared object files: 
export LD_LIBRARY_PATH=$HOME/opt/ThirdParty-13/platforms/linux64GccDPInt32/lib:LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/opt/OpenFOAM-13/platforms/linux64GccDPInt32Opt/lib:LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/opt/OpenFOAM-13/platforms/linux64GccDPInt32OptSYSTEMOPENMPI/lib:LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/opt/OpenFOAM-13/platforms/linux64GccDPInt32Opt/lib/openmpi-system:LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/opt/OpenFOAM-13/platforms/linux64GccDPInt32Opt/lib/dummy:LD_LIBRARY_PATH

/opt/slurm/bin/srun 2 ./Main 2 2

#https://stackoverflow.com/questions/38905391/how-can-i-run-mpi-job-in-multiple-nodes-multinode-mpi-job-execution


