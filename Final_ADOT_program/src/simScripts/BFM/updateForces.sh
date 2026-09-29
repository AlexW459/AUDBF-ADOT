#!/bin/bash
# First 3 arguments are the COM
# Next 3 arguments are the gravity vector (not normalised)
# Next argument is the velocity magnitude
# Next argument is the air density
# Final argument is the test number

#Enters case
caseNum="Aerodynamics_Simulation_BFM_Test_$9"
cd $caseNum

#Clear forces function file past line 18
#sed -i '18,$d' system/forces

#Clear velocity magnitudes function file past line 17
#sed -i '17,$d' system/velocityMagnitudes

#Set force coeffs values for aeroForces
sed -i "/rhoInf  /c\    rhoInf             $8;" system/forces
sed -i "/CofR/c\    CofR            ($1 $2 $3);" system/forces
sed -i "/magUInf/c\    magUInf         $7;" system/forces

#Set gravity vector
sed -i "/value/c\value           ($4 $5 $6);" constant/g

