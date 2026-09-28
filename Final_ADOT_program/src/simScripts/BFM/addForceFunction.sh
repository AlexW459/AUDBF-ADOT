#!/bin/bash

# First argument is the case number
# Second argument is the force region number
# Next argument is rho
# Next 3 arguments are the centre of rotation coordinates
# Next argument is the velocity magnitude

#Enters case
caseNum="Aerodynamics_Simulation_BFM_$1"
cd $caseNum
cd system


#Creates forces function objects
cat <<EOF >> forces
aeroForces_$2
{
    type            forces;
    libs            ("libforces.so");
    patches         (forcePatch_$2);
    
    rho             $3;

    CofR            ($4 $5 $6);

    magUInf         $7;

    writeFields     yes;
    writeControl outputTime;
    writeInterval 1;
    writeFormat     ascii;
}
EOF
