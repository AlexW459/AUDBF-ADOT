#!/bin/bash

# First argument is the case number
# Second argument is the velocity region number

caseNum="Aerodynamics_Simulation_BFM_$1"
cd $caseNum
cd system

#Creates velocity magnitude function objects
cat >> velocityMagnitudes <<EOF
velocity_$2
{
    type            volFieldValue;
    libs            ("libfieldFunctionObjects.so");

    regionType      zone;
    cellZone        velZone_$2;
    operation       volAverage;

    fields          (magU);

    writeFields     yes;
    writeControl    outputTime;
    writeFormat     ascii;
    writeInterval   1;
}
EOF