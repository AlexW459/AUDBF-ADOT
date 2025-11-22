#!/bin/bash

# First 3 arguments are direction of velocity
# Next 2 arguments are the sign of velocity in the y and z directions



#Updates inlet velocity
velocityLineNum="$(grep -n "flowVelocity" Aerodynamics_Simulation/initialValues/initialConditions | head -n 1 | cut -d: -f1)"
newVelocity="flowVelocity       ($1 $2 $3);"
sed -i "$((velocityLineNum))s/.*/$newVelocity/" Aerodynamics_Simulation/initialValues/initialConditions

#Updates patch types
#Inlets are type fixed value for velocity, and type zeroGradient for pressure
#Outlets are type inletOutlet for velocity (use inlet velocity as value), and type fixed value for pressure (use 0)
#Patches parallel to fluid flow should be treated as an outlet, but with (0, 0, 0) as the value

UfrontlineNum="$(grep -n "front" Aerodynamics_Simulation/initialValues/U | head -n 1 | cut -d: -f1)"
UbacklineNum="$(grep -n "back" Aerodynamics_Simulation/initialValues/U | head -n 1 | cut -d: -f1)"
pfrontlineNum="$(grep -n "front" Aerodynamics_Simulation/initialValues/p | head -n 1 | cut -d: -f1)"
pbacklineNum="$(grep -n "back" Aerodynamics_Simulation/initialValues/p | head -n 1 | cut -d: -f1)"

if [ $(echo "$4 > 0" | bc) ]; then
    #Set front to outlet
    sed -i "$((UfrontlineNum+2))s/.*/        type           inletOutlet;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UfrontlineNum+3))s/.*/        inletValue     uniform \$flowVelocity;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UfrontlineNum+4))s/.*/        value          uniform \$flowVelocity;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((pfrontlineNum+2))s/.*/        type           inletOutlet;/" Aerodynamics_Simulation/initialValues/p
    sed -i "$((pfrontlineNum+3))s/.*/        inletValue     uniform 0;/" Aerodynamics_Simulation/initialValues/p
    sed -i "$((pfrontlineNum+4))s/.*/        value          uniform 0;/" Aerodynamics_Simulation/initialValues/p
else
    #Set front to inlet
    sed -i "$((UfrontlineNum+2))s/.*/        type           fixedValue;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UfrontlineNum+3))s/.*/        value          uniform \$flowVelocity;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UfrontlineNum+4))s/.*/ /" Aerodynamics_Simulation/initialValues/U
    sed -i "$((pfrontlineNum+2))s/.*/        type           zeroGradient;/" Aerodynamics_Simulation/initialValues/p
    sed -i "$((pfrontlineNum+3))s/.*/ /" Aerodynamics_Simulation/initialValues/p
    sed -i "$((pfrontlineNum+4))s/.*/ /" Aerodynamics_Simulation/initialValues/p
fi

if [ $(echo "$4 < 0" | bc) ]; then
#Set back to outlet
    sed -i "$((UbacklineNum+2))s/.*/        type           inletOutlet;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UbacklineNum+3))s/.*/        inletValue     uniform \$flowVelocity;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UbacklineNum+4))s/.*/        value          uniform \$flowVelocity;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((pbacklineNum+2))s/.*/        type           inletOutlet;/" Aerodynamics_Simulation/initialValues/p
    sed -i "$((pbacklineNum+3))s/.*/        inletValue     uniform 0;/" Aerodynamics_Simulation/initialValues/p
    sed -i "$((pbacklineNum+4))s/.*/        value          uniform 0;/" Aerodynamics_Simulation/initialValues/p
else
    #Set back to inlet
    sed -i "$((UbacklineNum+2))s/.*/        type           fixedValue;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UbacklineNum+3))s/.*/        value          uniform \$flowVelocity;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UbacklineNum+4))s/.*/ /" Aerodynamics_Simulation/initialValues/U
    sed -i "$((pbacklineNum+2))s/.*/        type           zeroGradient;/" Aerodynamics_Simulation/initialValues/p
    sed -i "$((pbacklineNum+3))s/.*/ /" Aerodynamics_Simulation/initialValues/p
    sed -i "$((pbacklineNum+4))s/.*/ /" Aerodynamics_Simulation/initialValues/p
fi

UupperlineNum="$(grep -n "upper" Aerodynamics_Simulation/initialValues/U | head -n 1 | cut -d: -f1)"
UlowerlineNum="$(grep -n "lower" Aerodynamics_Simulation/initialValues/U | head -n 1 | cut -d: -f1)"
pupperlineNum="$(grep -n "upper" Aerodynamics_Simulation/initialValues/p | head -n 1 | cut -d: -f1)"
plowerlineNum="$(grep -n "lower" Aerodynamics_Simulation/initialValues/p | head -n 1 | cut -d: -f1)"

if [ $(echo "$5 >= 0" | bc) ]; then
    #Set upper to outlet
    sed -i "$((UupperlineNum+2))s/.*/        type           inletOutlet;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UupperlineNum+3))s/.*/        inletValue     uniform \$flowVelocity;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UupperlineNum+4))s/.*/        value          uniform \$flowVelocity;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((pupperlineNum+2))s/.*/        type           inletOutlet;/" Aerodynamics_Simulation/initialValues/p
    sed -i "$((pupperlineNum+3))s/.*/        inletValue     uniform 0;/" Aerodynamics_Simulation/initialValues/p
    sed -i "$((pupperlineNum+4))s/.*/        value          uniform 0;/" Aerodynamics_Simulation/initialValues/p
else
    #Set upper to inlet
    sed -i "$((UupperlineNum+2))s/.*/        type           fixedValue;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UupperlineNum+3))s/.*/        value          uniform \$flowVelocity;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UupperlineNum+4))s/.*/ /" Aerodynamics_Simulation/initialValues/U
    sed -i "$((pupperlineNum+2))s/.*/        type           zeroGradient;/" Aerodynamics_Simulation/initialValues/p
    sed -i "$((pupperlineNum+3))s/.*/ /" Aerodynamics_Simulation/initialValues/p
    sed -i "$((pupperlineNum+4))s/.*/ /" Aerodynamics_Simulation/initialValues/p
fi

if [ $(echo "$5 <= 0" | bc) ]; then
    #Set lower to outlet
    sed -i "$((UlowerlineNum+2))s/.*/        type           inletOutlet;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UlowerlineNum+3))s/.*/        inletValue     uniform \$flowVelocity;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UlowerlineNum+4))s/.*/        value          uniform \$flowVelocity;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((plowerlineNum+2))s/.*/        type           inletOutlet;/" Aerodynamics_Simulation/initialValues/p
    sed -i "$((plowerlineNum+3))s/.*/        inletValue     uniform 0;/" Aerodynamics_Simulation/initialValues/p
    sed -i "$((plowerlineNum+4))s/.*/        value          uniform 0;/" Aerodynamics_Simulation/initialValues/p
else
    #Set lower to inlet
    sed -i "$((UlowerlineNum+2))s/.*/        type           fixedValue;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UlowerlineNum+3))s/.*/        value          uniform \$flowVelocity;/" Aerodynamics_Simulation/initialValues/U
    sed -i "$((UlowerlineNum+4))s/.*/ /" Aerodynamics_Simulation/initialValues/U
    sed -i "$((plowerlineNum+2))s/.*/        type           zeroGradient;/" Aerodynamics_Simulation/initialValues/p
    sed -i "$((plowerlineNum+3))s/.*/ /" Aerodynamics_Simulation/initialValues/p
    sed -i "$((plowerlineNum+4))s/.*/ /" Aerodynamics_Simulation/initialValues/p
fi
