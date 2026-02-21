#!/bin/bash

# First 3 arguments are velocity in x, y and z direction
# Next argument is the surface height deviation
# Next argument is the specific turbulence dissipation rate
# Final argument is the case number

#Enters case
caseNum="Aerodynamics_Simulation_$6"
cd $caseNum

#Updates inlet velocity
velocityLineNum="$(grep -n "flowVelocity" initialValues/initialConditions | head -n 1 | cut -d: -f1)"
newVelocity="flowVelocity       ($1 $2 $3);"
sed -i "$((velocityLineNum))s/.*/$newVelocity/" initialValues/initialConditions

#Updates patch types
#Inlets are type fixed value for velocity, and type zeroGradient for pressure
#Outlets are type inletOutlet for velocity (use inlet velocity as value), and type fixed value for pressure (use 0)
#Patches parallel to fluid flow should be treated as an outlet, but with (0, 0, 0) as the value


UupperlineNum="$(grep -n "upper" initialValues/U | head -n 1 | cut -d: -f1)"
UlowerlineNum="$(grep -n "lower" initialValues/U | head -n 1 | cut -d: -f1)"
pupperlineNum="$(grep -n "upper" initialValues/p | head -n 1 | cut -d: -f1)"
plowerlineNum="$(grep -n "lower" initialValues/p | head -n 1 | cut -d: -f1)"


if [[ $(echo "$3 >=  0" | bc) == "1" ]]; then
    #Set upper to outlet
    sed -i "$((UupperlineNum+2))s/.*/        type           inletOutlet;/" initialValues/U
    sed -i "$((UupperlineNum+3))s/.*/        inletValue     uniform \$flowVelocity;/" initialValues/U
    sed -i "$((UupperlineNum+4))s/.*/        value          uniform \$flowVelocity;/" initialValues/U
    sed -i "$((pupperlineNum+2))s/.*/        type           inletOutlet;/" initialValues/p
    sed -i "$((pupperlineNum+3))s/.*/        inletValue     uniform 0;/" initialValues/p
    sed -i "$((pupperlineNum+4))s/.*/        value          uniform 0;/" initialValues/p
else
    #Set upper to inlet
    sed -i "$((UupperlineNum+2))s/.*/        type           fixedValue;/" initialValues/U
    sed -i "$((UupperlineNum+3))s/.*/        value          uniform \$flowVelocity;/" initialValues/U
    sed -i "$((UupperlineNum+4))s/.*/ /" initialValues/U
    sed -i "$((pupperlineNum+2))s/.*/        type           zeroGradient;/" initialValues/p
    sed -i "$((pupperlineNum+3))s/.*/ /" initialValues/p
    sed -i "$((pupperlineNum+4))s/.*/ /" initialValues/p
fi

if [[ $(echo "$3 <= 0" | bc) == "1" ]]; then
    #Set lower to outlet
    sed -i "$((UlowerlineNum+2))s/.*/        type           inletOutlet;/" initialValues/U
    sed -i "$((UlowerlineNum+3))s/.*/        inletValue     uniform \$flowVelocity;/" initialValues/U
    sed -i "$((UlowerlineNum+4))s/.*/        value          uniform \$flowVelocity;/" initialValues/U
    sed -i "$((plowerlineNum+2))s/.*/        type           inletOutlet;/" initialValues/p
    sed -i "$((plowerlineNum+3))s/.*/        inletValue     uniform 0;/" initialValues/p
    sed -i "$((plowerlineNum+4))s/.*/        value          uniform 0;/" initialValues/p
else
    #Set lower to inlet
    sed -i "$((UlowerlineNum+2))s/.*/        type           fixedValue;/" initialValues/U
    sed -i "$((UlowerlineNum+3))s/.*/        value          uniform \$flowVelocity;/" initialValues/U
    sed -i "$((UlowerlineNum+4))s/.*/ /" initialValues/U
    sed -i "$((plowerlineNum+2))s/.*/        type           zeroGradient;/" initialValues/p
    sed -i "$((plowerlineNum+3))s/.*/ /" initialValues/p
    sed -i "$((plowerlineNum+4))s/.*/ /" initialValues/p
fi

#Set surface roughness
nutWallLineNum="$(grep -n "aircraftModel" initialValues/nut | head -n 1 | cut -d: -f1)"
sed -i "$((nutWallLineNum+4))s/.*/        Ks              $4;/" initialValues/nut

#Set specific turbulence dissipation rate
omegaLineNum="$(grep -n "specificTurbulenceDissipationRate" initialValues/initialConditions | head -n 1 | cut -d: -f1)"
sed -i "$((omegaLineNum))s/.*/specificTurbulenceDissipationRate $5;/" initialValues/initialConditions

rm -r 0/*

cp -a initialValues/. 0/

cp -r constant/polyMesh 0/