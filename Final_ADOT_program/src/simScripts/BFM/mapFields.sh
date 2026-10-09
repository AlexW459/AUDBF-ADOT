#!/bin/bash

# First arguments is openfoam source 
# Second argument is the position number
# Third argument is the test number
# Next 6 arguments are whether to include patches in cutting patches:
# Neg x, pos x, neg y, pos y, neg z, pos z
# Final argument is the time to map from the source case

#Loads latest Openfoam version
openfoamSource=$1

#. $openfoamSource

#Enters case
caseNum="Aerodynamics_Simulation_BFM_Test_$3"
cd $caseNum

cpLineNum="$(grep -n "cuttingPatches" system/mapFieldsDict | head -n 1 | cut -d: -f1)"

# Cut in neg x
if [[ $4 -gt 0 ]]; then
    sed -i "$((cpLineNum+2))s/.*/   outlet/" system/mapFieldsDict
else
    sed -i "$((cpLineNum+2))s/.*/ /" system/mapFieldsDict
fi

# Cut in pos x
if [[ $5 -gt 0 ]]; then
    sed -i "$((cpLineNum+3))s/.*/   inlet/" system/mapFieldsDict
else
    sed -i "$((cpLineNum+3))s/.*/ /" system/mapFieldsDict
fi

# Cut in neg y
if [[ $6 -gt 0 ]]; then
    sed -i "$((cpLineNum+4))s/.*/   back/" system/mapFieldsDict
else
    sed -i "$((cpLineNum+4))s/.*/ /" system/mapFieldsDict
fi

# Cut in pos y
if [[ $7 -gt 0 ]]; then
    sed -i "$((cpLineNum+5))s/.*/   front/" system/mapFieldsDict
else
    sed -i "$((cpLineNum+5))s/.*/ /" system/mapFieldsDict
fi

# Cut in neg z
if [[ $8 -gt 0 ]]; then
    sed -i "$((cpLineNum+6))s/.*/   lower/" system/mapFieldsDict
else
    sed -i "$((cpLineNum+6))s/.*/ /" system/mapFieldsDict
fi

# Cut in pos z
if [[ $9 -gt 0 ]]; then
    sed -i "$((cpLineNum+7))s/.*/   upper/" system/mapFieldsDict
else
    sed -i "$((cpLineNum+7))s/.*/ /" system/mapFieldsDict
fi

mapFields ../Aerodynamics_Simulation_BFM_$2 -sourceTime ${10} > mapLog
