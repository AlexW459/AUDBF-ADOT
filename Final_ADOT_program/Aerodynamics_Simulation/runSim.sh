#!/bin/bash

#First argument is endTime, second argument is timeStep

cd Aerodynamics_Simulation


. /opt/openfoam13/etc/bashrc

#Deletes all previous time directories after zero
rm -r *0.*

#mpirun potentialFoam -initialiseUBCs -parallel -writep

#Sets controls for simulation
endTimeLineNum="$(grep -n "endTime" system/controlDict | head -n 1 | cut -d: -f1)"
deltaTLineNum="$(grep -n "deltaT" system/controlDict | head -n 1 | cut -d: -f1)"

sed -i "$((endTimeLineNum))s/.*/endTime         $1;/" system/controlDict
sed -i "$((deltaTLineNum))s/.*/deltaT          $2;/" system/controlDict

potentialFoam -writep

foamRun -solver incompressibleFluid > simLog


#reconstructPar -withZero

#rm -r  processor*
