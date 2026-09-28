#!/bin/bash

# First argument is the case number
# Second argument is the velocity region number
# Next 6 arguments are the bounds of the velocity region

#Enters case
caseNum="Aerodynamics_Simulation_BFM_$1"
cd $caseNum
cd system

# Adds regions to isolate velocity
cat >> createZonesDict<<EOF
box{
    name velZone_$2
    zoneType cell;
    type box;
    box ($3 $4 $5 $6 $7 $8);
};
EOF

