#!/bin/bash

# First 3 arguments are velocity in x, y and z direction
# Next argument is the test number


# Next argument is the air density
# Next argument is surface roughness height
# Next argument is the turbulence energy
# Next argument is the specific turbulence dissipation rate
# Next argument is the test number

#Enters case
caseNum="Aerodynamics_Simulation_BFM_Test_$4"
cd $caseNum

#Updates inlet velocity
sed -i "/flowVelocity/c\flowVelocity       ($1 $2 $3);" initialValues/initialConditions

#sed -i "/surfaceRoughnessHeight/c\surfaceRoughnessHeight       $5;" initialValues/initialConditions
#sed -i "/turbulentEnergy/c\turbulentEnergy       $6;" initialValues/initialConditions
#sed -i "/specificTurbulenceDissipationRate/c\specificTurbulenceDissipationRate       $7;" initialValues/initialConditions

#Updates patch types
#Inlets are type fixed value for velocity, and type zeroGradient for pressure
#Outlets are type inletOutlet for velocity (use inlet velocity as value), and type fixed value for pressure (use 0)
#Patches parallel to fluid flow should be treated as an outlet, but with (0, 0, 0) as the value


UupperlineNum="$(grep -n "upper" initialValues/U | head -n 1 | cut -d: -f1)"
UlowerlineNum="$(grep -n "lower" initialValues/U | head -n 1 | cut -d: -f1)"
UfrontlineNum="$(grep -n "front" initialValues/U | head -n 1 | cut -d: -f1)"
UbacklineNum="$(grep -n "back" initialValues/U | head -n 1 | cut -d: -f1)"
UinletlineNum="$(grep -n "inlet" initialValues/U | head -n 1 | cut -d: -f1)"
UoutletlineNum="$(grep -n "outlet" initialValues/U | head -n 1 | cut -d: -f1)"

pupperlineNum="$(grep -n "upper" initialValues/p | head -n 1 | cut -d: -f1)"
plowerlineNum="$(grep -n "lower" initialValues/p | head -n 1 | cut -d: -f1)"
pfrontlineNum="$(grep -n "front" initialValues/p | head -n 1 | cut -d: -f1)"
pbacklineNum="$(grep -n "back" initialValues/p | head -n 1 | cut -d: -f1)"
pinletlineNum="$(grep -n "inlet" initialValues/p | head -n 1 | cut -d: -f1)"
poutletlineNum="$(grep -n "outlet" initialValues/p | head -n 1 | cut -d: -f1)"


#OinletlineNum="$(grep -n "inlet" initialValues/omega | head -n 1 | cut -d: -f1)"
#OoutletlineNum="$(grep -n "outlet" initialValues/omega | head -n 1 | cut -d: -f1)"
#OfrontlineNum="$(grep -n "front" initialValues/omega | head -n 1 | cut -d: -f1)"
#ObacklineNum="$(grep -n "back" initialValues/omega | head -n 1 | cut -d: -f1)"
#OupperlineNum="$(grep -n "upper" initialValues/omega | head -n 1 | cut -d: -f1)"
#OlowerlineNum="$(grep -n "lower" initialValues/omega | head -n 1 | cut -d: -f1)"

#kinletlineNum="$(grep -n "inlet" initialValues/k | head -n 1 | cut -d: -f1)"
#koutletlineNum="$(grep -n "outlet" initialValues/k | head -n 1 | cut -d: -f1)"
#kfrontlineNum="$(grep -n "front" initialValues/k | head -n 1 | cut -d: -f1)"
#kbacklineNum="$(grep -n "back" initialValues/k | head -n 1 | cut -d: -f1)"
#kupperlineNum="$(grep -n "upper" initialValues/k | head -n 1 | cut -d: -f1)"
#klowerlineNum="$(grep -n "lower" initialValues/k | head -n 1 | cut -d: -f1)"


if [[ $(echo "$3 >=  0" | bc) == "1" ]]; then
    #Set upper to outlet
    sed -i "$((UupperlineNum+2))s/.*/        type           inletOutlet;/" initialValues/U
    sed -i "$((UupperlineNum+3))s/.*/        inletValue     uniform \$flowVelocity;/" initialValues/U
    sed -i "$((UupperlineNum+4))s/.*/        value          uniform \$flowVelocity;/" initialValues/U
    sed -i "$((pupperlineNum+2))s/.*/        type           inletOutlet;/" initialValues/p
    sed -i "$((pupperlineNum+3))s/.*/        inletValue     uniform 0;/" initialValues/p
    sed -i "$((pupperlineNum+4))s/.*/        value          uniform 0;/" initialValues/p
    #sed -i "$((OupperlineNum+2))s/.*/        type           inletOutlet;/" initialValues/omega
    #sed -i "$((OupperlineNum+3))s/.*/        inletValue     uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((OupperlineNum+4))s/.*/        value          uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((kupperlineNum+2))s/.*/        type           inletOutlet;/" initialValues/k
    #sed -i "$((kupperlineNum+3))s/.*/        inletValue     uniform \$turbulentEnergy;/" initialValues/k
    #sed -i "$((kupperlineNum+4))s/.*/        value          uniform \$turbulentEnergy;/" initialValues/k

else
    #Set upper to inlet
    sed -i "$((UupperlineNum+2))s/.*/        type           fixedValue;/" initialValues/U
    sed -i "$((UupperlineNum+3))s/.*/        value          uniform \$flowVelocity;/" initialValues/U
    sed -i "$((UupperlineNum+4))s/.*/ /" initialValues/U
    sed -i "$((pupperlineNum+2))s/.*/        type           zeroGradient;/" initialValues/p
    sed -i "$((pupperlineNum+3))s/.*/ /" initialValues/p
    sed -i "$((pupperlineNum+4))s/.*/ /" initialValues/p
    #sed -i "$((OupperlineNum+2))s/.*/        type           fixedValue;/" initialValues/omega
    #sed -i "$((OupperlineNum+3))s/.*/        value          uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((OupperlineNum+4))s/.*/ /" initialValues/omega
    #sed -i "$((kupperlineNum+2))s/.*/        type           fixedValue;/" initialValues/k
    #sed -i "$((kupperlineNum+3))s/.*/        value          uniform \$turbulentEnergy;/" initialValues/k
    #sed -i "$((kupperlineNum+4))s/.*/ /" initialValues/k
        
fi

if [[ $(echo "$3 <= 0" | bc) == "1" ]]; then
    #Set lower to outlet
    sed -i "$((UlowerlineNum+2))s/.*/        type           zeroGradient;/" initialValues/U
    sed -i "$((UlowerlineNum+3))s/.*/ /" initialValues/U
    sed -i "$((UlowerlineNum+4))s/.*/ /" initialValues/U
    sed -i "$((plowerlineNum+2))s/.*/        type           fixedValue;/" initialValues/p
    sed -i "$((plowerlineNum+3))s/.*/        value          uniform 0;/" initialValues/p
    sed -i "$((plowerlineNum+4))s/.*/ /" initialValues/p
    #sed -i "$((OlowerlineNum+2))s/.*/        type           inletOutlet;/" initialValues/omega
    #sed -i "$((OlowerlineNum+3))s/.*/        inletValue     uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((OlowerlineNum+4))s/.*/        value          uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((klowerlineNum+2))s/.*/        type           inletOutlet;/" initialValues/k
    #sed -i "$((klowerlineNum+3))s/.*/        inletValue     uniform \$turbulentEnergy;/" initialValues/k
    #sed -i "$((klowerlineNum+4))s/.*/        value          uniform \$turbulentEnergy;/" initialValues/k
    
else
    #Set lower to inlet
    sed -i "$((UlowerlineNum+2))s/.*/        type           fixedValue;/" initialValues/U
    sed -i "$((UlowerlineNum+3))s/.*/        value          uniform \$flowVelocity;/" initialValues/U
    sed -i "$((UlowerlineNum+4))s/.*/ /" initialValues/U
    sed -i "$((plowerlineNum+2))s/.*/        type           zeroGradient;/" initialValues/p
    sed -i "$((plowerlineNum+3))s/.*/ /" initialValues/p
    sed -i "$((plowerlineNum+4))s/.*/ /" initialValues/p
    #sed -i "$((OlowerlineNum+2))s/.*/        type           fixedValue;/" initialValues/omega
    #sed -i "$((OlowerlineNum+3))s/.*/        value          uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((OlowerlineNum+4))s/.*/ /" initialValues/omega
    #sed -i "$((klowerlineNum+2))s/.*/        type           fixedValue;/" initialValues/k
    #sed -i "$((klowerlineNum+3))s/.*/        value          uniform \$turbulentEnergy;/" initialValues/k
    #sed -i "$((klowerlineNum+4))s/.*/ /" initialValues/k
fi



if [[ $(echo "$2 >=  0" | bc) == "1" ]]; then
    #Set front to outlet
    sed -i "$((UfrontlineNum+2))s/.*/        type           zeroGradient;/" initialValues/U
    sed -i "$((UfrontlineNum+3))s/.*/ /" initialValues/U
    sed -i "$((UfrontlineNum+4))s/.*/ /" initialValues/U
    sed -i "$((pfrontlineNum+2))s/.*/        type           fixedValue;/" initialValues/p
    sed -i "$((pfrontlineNum+3))s/.*/        value          uniform 0;/" initialValues/p
    sed -i "$((pfrontlineNum+4))s/.*/ /" initialValues/p
    #sed -i "$((OfrontlineNum+2))s/.*/        type           inletOutlet;/" initialValues/omega
    #sed -i "$((OfrontlineNum+3))s/.*/        inletValue     uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((OfrontlineNum+4))s/.*/        value          uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((kfrontlineNum+2))s/.*/        type           inletOutlet;/" initialValues/k
    #sed -i "$((kfrontlineNum+3))s/.*/        inletValue     uniform \$turbulentEnergy;/" initialValues/k
    #sed -i "$((kfrontlineNum+4))s/.*/        value          uniform \$turbulentEnergy;/" initialValues/k
else
    #Set front to inlet
    sed -i "$((UfrontlineNum+2))s/.*/        type           fixedValue;/" initialValues/U
    sed -i "$((UfrontlineNum+3))s/.*/        value          uniform \$flowVelocity;/" initialValues/U
    sed -i "$((UfrontlineNum+4))s/.*/ /" initialValues/U
    sed -i "$((pfrontlineNum+2))s/.*/        type           zeroGradient;/" initialValues/p
    sed -i "$((pfrontlineNum+3))s/.*/ /" initialValues/p
    sed -i "$((pfrontlineNum+4))s/.*/ /" initialValues/p
    #sed -i "$((OfrontlineNum+2))s/.*/        type           fixedValue;/" initialValues/omega
    #sed -i "$((OfrontlineNum+3))s/.*/        value          uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((OfrontlineNum+4))s/.*/ /" initialValues/omega
    #sed -i "$((kfrontlineNum+2))s/.*/        type           fixedValue;/" initialValues/k
    #sed -i "$((kfrontlineNum+3))s/.*/        value          uniform \$turbulentEnergy;/" initialValues/k
    #sed -i "$((kfrontlineNum+4))s/.*/ /" initialValues/k
        
fi

if [[ $(echo "$2 <= 0" | bc) == "1" ]]; then
    #Set back to outlet
    sed -i "$((UbacklineNum+2))s/.*/        type           zeroGradient;/" initialValues/U
    sed -i "$((UbacklineNum+3))s/.*/ /" initialValues/U
    sed -i "$((UbacklineNum+4))s/.*/ /" initialValues/U
    sed -i "$((pbacklineNum+2))s/.*/        type           fixedValue;/" initialValues/p
    sed -i "$((pbacklineNum+3))s/.*/        value          uniform 0;/" initialValues/p
    sed -i "$((pbacklineNum+4))s/.*/ /" initialValues/p
    #sed -i "$((ObacklineNum+2))s/.*/        type           inletOutlet;/" initialValues/omega
    #sed -i "$((ObacklineNum+3))s/.*/        inletValue     uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((ObacklineNum+4))s/.*/        value          uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((kbacklineNum+2))s/.*/        type          inletOutlet;/" initialValues/k
    #sed -i "$((kbacklineNum+3))s/.*/        inletValue     uniform \$turbulentEnergy;/" initialValues/k
    #sed -i "$((kbacklineNum+4))s/.*/        value          uniform \$turbulentEnergy;/" initialValues/k
else
    #Set back to inlet
    sed -i "$((UbacklineNum+2))s/.*/        type           fixedValue;/" initialValues/U
    sed -i "$((UbacklineNum+3))s/.*/        value          uniform \$flowVelocity;/" initialValues/U
    sed -i "$((UbacklineNum+4))s/.*/ /" initialValues/U
    sed -i "$((pbacklineNum+2))s/.*/        type           zeroGradient;/" initialValues/p
    sed -i "$((pbacklineNum+3))s/.*/ /" initialValues/p
    sed -i "$((pbacklineNum+4))s/.*/ /" initialValues/p
    #sed -i "$((ObacklineNum+2))s/.*/        type           fixedValue;/" initialValues/omega
    #sed -i "$((ObacklineNum+3))s/.*/        value          uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((ObacklineNum+4))s/.*/ /" initialValues/omega
    #sed -i "$((kbacklineNum+2))s/.*/        type           fixedValue;/" initialValues/k
    #sed -i "$((kbacklineNum+3))s/.*/        value          uniform \$turbulentEnergy;/" initialValues/k
    #sed -i "$((kbacklineNum+4))s/.*/ /" initialValues/k
        
fi


if [[ $(echo "$1 >=  0" | bc) == "1" ]]; then
    #Set inlet to outlet
    sed -i "$((UinletlineNum+2))s/.*/        type           zeroGradient;/" initialValues/U
    sed -i "$((UinletlineNum+3))s/.*/ /" initialValues/U
    sed -i "$((UinletlineNum+4))s/.*/ /" initialValues/U
    sed -i "$((pinletlineNum+2))s/.*/        type           inletOutlet;/" initialValues/p
    sed -i "$((pinletlineNum+3))s/.*/        inletValue     uniform 0;/" initialValues/p
    sed -i "$((pinletlineNum+4))s/.*/        value          uniform 0;/" initialValues/p
    #sed -i "$((OinletlineNum+2))s/.*/        type           inletOutlet;/" initialValues/omega
    #sed -i "$((OinletlineNum+3))s/.*/        inletValue     uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((OinletlineNum+4))s/.*/        value          uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((kinletlineNum+2))s/.*/        type           inletOutlet;/" initialValues/k
    #sed -i "$((kinletlineNum+3))s/.*/        inletValue     uniform \$turbulentEnergy;/" initialValues/k
    #sed -i "$((kinletlineNum+4))s/.*/        value          uniform \$turbulentEnergy;/" initialValues/k
else
    #Set inlet to inlet
    sed -i "$((UinletlineNum+2))s/.*/        type           fixedValue;/" initialValues/U
    sed -i "$((UinletlineNum+3))s/.*/        value          uniform \$flowVelocity;/" initialValues/U
    sed -i "$((UinletlineNum+4))s/.*/ /" initialValues/U
    sed -i "$((pinletlineNum+2))s/.*/        type           zeroGradient;/" initialValues/p
    sed -i "$((pinletlineNum+3))s/.*/ /" initialValues/p
    sed -i "$((pinletlineNum+4))s/.*/ /" initialValues/p
    #sed -i "$((OinletlineNum+2))s/.*/        type           fixedValue;/" initialValues/omega
    #sed -i "$((OinletlineNum+3))s/.*/        value          uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((OinletlineNum+4))s/.*/ /" initialValues/omega
    #sed -i "$((kinletlineNum+2))s/.*/        type           fixedValue;/" initialValues/k
    #sed -i "$((kinletlineNum+3))s/.*/        value          uniform \$turbulentEnergy;/" initialValues/k
    #sed -i "$((kinletlineNum+4))s/.*/ /" initialValues/k
        
fi

if [[ $(echo "$1 <= 0" | bc) == "1" ]]; then
    #Set outlet to outlet
    sed -i "$((UoutletlineNum+2))s/.*/        type           zeroGradient;/" initialValues/U
    sed -i "$((UoutletlineNum+3))s/.*/ /" initialValues/U
    sed -i "$((UoutletlineNum+4))s/.*/ /" initialValues/U
    sed -i "$((poutletlineNum+2))s/.*/        type           fixedValue;/" initialValues/p
    sed -i "$((poutletlineNum+3))s/.*/        value     uniform 0;/" initialValues/p
    sed -i "$((poutletlineNum+4))s/.*/ /" initialValues/p
    #sed -i "$((OoutletlineNum+2))s/.*/        type           inletOutlet;/" initialValues/omega
    #sed -i "$((OoutletlineNum+3))s/.*/        inletValue     uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((OoutletlineNum+4))s/.*/        value          uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((koutletlineNum+2))s/.*/        type           inletOutlet;/" initialValues/k
    #sed -i "$((koutletlineNum+3))s/.*/        inletValue     uniform \$turbulentEnergy;/" initialValues/k
    #sed -i "$((koutletlineNum+4))s/.*/        value          uniform \$turbulentEnergy;/" initialValues/k
else
    #Set outlet to inlet
    sed -i "$((UoutletlineNum+2))s/.*/        type           fixedValue;/" initialValues/U
    sed -i "$((UoutletlineNum+3))s/.*/        value          uniform \$flowVelocity;/" initialValues/U
    sed -i "$((UoutletlineNum+4))s/.*/ /" initialValues/U
    sed -i "$((poutletlineNum+2))s/.*/        type           zeroGradient;/" initialValues/p
    sed -i "$((poutletlineNum+3))s/.*/ /" initialValues/p
    sed -i "$((poutletlineNum+4))s/.*/ /" initialValues/p
    #sed -i "$((OoutletlineNum+2))s/.*/        type           fixedValue;/" initialValues/omega
    #sed -i "$((OoutletlineNum+3))s/.*/        value          uniform \$specificTurbulenceDissipationRate;/" initialValues/omega
    #sed -i "$((OoutletlineNum+4))s/.*/ /" initialValues/omega
    #sed -i "$((koutletlineNum+2))s/.*/        type           fixedValue;/" initialValues/k
    #sed -i "$((koutletlineNum+3))s/.*/        value          uniform \$turbulentEnergy;/" initialValues/k
    #sed -i "$((koutletlineNum+4))s/.*/ /" initialValues/k
        
fi

rm -r -f 0/*
rm -r -f 0.* 1.* *e-**

cp -a initialValues/. 0/

cp -r constant/polyMesh 0/