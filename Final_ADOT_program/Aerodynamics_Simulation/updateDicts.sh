
# First 6 arguments are the 3 min bounds and 3 max bounds of the volume
# Next 3 arguments are direction of velocity
# Next 2 arguments are the sign of velocity in the y and z directions
# Next 6 arguments are the 3 min bounds and 3 max bounds of the refinement box

#Updates bounding box
verticesLineNum="$(grep -n "vertices" Aerodynamics_Simulation/system/blockMeshDict | head -n 1 | cut -d: -f1)"

begin1=$((verticesLineNum+2))

vertex0="   ($1 $2 $3)"
vertex1="   ($4 $2 $3)"
vertex2="   ($4 $5 $3)"
vertex3="   ($1 $5 $3)"
vertex4="   ($1 $2 $6)"
vertex5="   ($4 $2 $6)"
vertex6="   ($4 $5 $6)"
vertex7="   ($1 $5 $6)"

sed -i "$((verticesLineNum+2))s/.*/$vertex0/" Aerodynamics_Simulation/system/blockMeshDict
sed -i "$((verticesLineNum+3))s/.*/$vertex1/" Aerodynamics_Simulation/system/blockMeshDict
sed -i "$((verticesLineNum+4))s/.*/$vertex2/" Aerodynamics_Simulation/system/blockMeshDict
sed -i "$((verticesLineNum+5))s/.*/$vertex3/" Aerodynamics_Simulation/system/blockMeshDict
sed -i "$((verticesLineNum+7))s/.*/$vertex4/" Aerodynamics_Simulation/system/blockMeshDict
sed -i "$((verticesLineNum+8))s/.*/$vertex5/" Aerodynamics_Simulation/system/blockMeshDict
sed -i "$((verticesLineNum+9))s/.*/$vertex6/" Aerodynamics_Simulation/system/blockMeshDict
sed -i "$((verticesLineNum+10))s/.*/$vertex7/" Aerodynamics_Simulation/system/blockMeshDict

#Updates refinementBox
refinementBoxLineNum="$(grep -n "refinementBox" Aerodynamics_Simulation/system/snappyHexMeshDict | head -n 1 | cut -d: -f1)"
minRefine="        min (${12} ${13} ${14});"
maxRefine="        max (${15} ${16} ${17});"
sed -i "$((refinementBoxLineNum+3))s/.*/$minRefine/" Aerodynamics_Simulation/system/snappyHexMeshDict
sed -i "$((refinementBoxLineNum+4))s/.*/$maxRefine/" Aerodynamics_Simulation/system/snappyHexMeshDict

#Sets the location of the inside point
insidePointLineNum="$(grep -n "insidePoint" Aerodynamics_Simulation/system/snappyHexMeshDict | head -n 1 | cut -d: -f1)"
insidePointX=$(echo "$12+0.001" | bc)
insidePoint="    insidePoint ($insidePointX 0.001 0.001);"
sed -i "$((insidePointLineNum))s/.*/$insidePoint/" Aerodynamics_Simulation/system/snappyHexMeshDict


#Updates inlet velocity
velocityLineNum="$(grep -n "flowVelocity" Aerodynamics_Simulation/initialValues/initialConditions | head -n 1 | cut -d: -f1)"
newVelocity="flowVelocity       ($7 $8 $9);"
sed -i "$((velocityLineNum))s/.*/$newVelocity/" Aerodynamics_Simulation/initialValues/initialConditions

#Updates patch types
#Inlets are type fixed value for velocity, and type zeroGradient for pressure
#Outlets are type inletOutlet for velocity (use inlet velocity as value), and type fixed value for pressure (use 0)
#Patches parallel to fluid flow should be treated as an outlet, but with (0, 0, 0) as the value

UfrontlineNum="$(grep -n "front" Aerodynamics_Simulation/initialValues/U | head -n 1 | cut -d: -f1)"
UbacklineNum="$(grep -n "back" Aerodynamics_Simulation/initialValues/U | head -n 1 | cut -d: -f1)"
pfrontlineNum="$(grep -n "front" Aerodynamics_Simulation/initialValues/p | head -n 1 | cut -d: -f1)"
pbacklineNum="$(grep -n "back" Aerodynamics_Simulation/initialValues/p | head -n 1 | cut -d: -f1)"

if [ "${10}" -ge "0" ]; then
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

if [ "${10}" -le "0" ]; then
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

if [ "${11}" -ge "0" ]; then
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

if [ "${11}" -le "0" ]; then
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



