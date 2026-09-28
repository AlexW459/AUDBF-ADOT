#!/bin/bash
# First 3 arguments are the gravity vector (not normalised)
# Next argument is the pos number
# Final argument is the test number

#Enters case
caseNum="Aerodynamics_Simulation_IBM_${4}_$5"
cd $caseNum

#Set gravity vector
sed -i "/gravity/c\    gravity ($1 $2 $3);" solidDict
