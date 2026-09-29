#!/bin/bash
# First 3 arguments are the gravity vector (not normalised)
# Final argument is the test number

#Enters case
caseNum="Aerodynamics_Simulation_IBM_Test_$4"
cd $caseNum

#Set gravity vector
sed -i "/gravity/c\    gravity ($1 $2 $3);" solidDict
