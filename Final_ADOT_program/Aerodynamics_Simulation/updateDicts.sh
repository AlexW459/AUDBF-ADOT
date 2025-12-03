#!/bin/bash

# First 3 arguments are velocity in x and z direction
# Next argument is the sign of velocity in the y direciton
# Next argument is the sign of velocity in the z directions
# Next argument is the surface height deviation
# Next argument is the specific turbulence dissipation rate

#Updates inlet velocity
velocityLineNum="$(grep -n "flowVelocity" Aerodynamics_Simulation/initialValues/initialConditions | head -n 1 | cut -d: -f1)"
newVelocity="flowVelocity       ($1 $2 $3);"
sed -i "$((velocityLineNum))s/.*/$newVelocity/" Aerodynamics_Simulation/initialValues/initialConditions

#Updates patch types
#Inlets are type fixed value for velocity, and type zeroGradient for pressure
#Outlets are type inletOutlet for velocity (use inlet velocity as value), and type fixed value for pressure (use 0)
#Patches parallel to fluid flow should be treated as an outlet, but with (0, 0, 0) as the value


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

#Set surface roughness
nutWallLineNum="$(grep -n "aircraftModel" Aerodynamics_Simulation/initialValues/nut | head -n 1 | cut -d: -f1)"
sed -i "$((nutWallLineNum+4))s/.*/        Ks              $6;/" Aerodynamics_Simulation/initialValues/nut

#Set specific turbulence dissipation rate
omegaLineNum="$(grep -n "specificTurbulenceDissipationRate" Aerodynamics_Simulation/initialValues/initialConditions | head -n 1 | cut -d: -f1)"
sed -i "$((omegaLineNum))s/.*/specificTurbulenceDissipationRate $7;/" Aerodynamics_Simulation/initialValues/initialConditions
