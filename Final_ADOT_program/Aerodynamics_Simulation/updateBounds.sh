#!/bin/bash

# First 6 arguments are the 3 min bounds and 3 max bounds of the volume
# Next 6 arguments are the 3 min bounds and 3 max bounds of the refinement box
# Next 6 arguments are the 3 min bounds and 3 max bounds of the horizontal stabiliser
# Final argument is the case number

#Enters case
caseNum="Aerodynamics_Simulation_${19}"
cd $caseNum

#Updates bounding box
verticesLineNum="$(grep -n "vertices" system/blockMeshDict | head -n 1 | cut -d: -f1)"

begin1=$((verticesLineNum+2))

vertex0="   ($1 $2 $3)"
vertex1="   ($4 $2 $3)"
vertex2="   ($4 $5 $3)"
vertex3="   ($1 $5 $3)"
vertex4="   ($1 $2 $6)"
vertex5="   ($4 $2 $6)"
vertex6="   ($4 $5 $6)"
vertex7="   ($1 $5 $6)"

sed -i "$((verticesLineNum+2))s/.*/$vertex0/" system/blockMeshDict
sed -i "$((verticesLineNum+3))s/.*/$vertex1/" system/blockMeshDict
sed -i "$((verticesLineNum+4))s/.*/$vertex2/" system/blockMeshDict
sed -i "$((verticesLineNum+5))s/.*/$vertex3/" system/blockMeshDict
sed -i "$((verticesLineNum+7))s/.*/$vertex4/" system/blockMeshDict
sed -i "$((verticesLineNum+8))s/.*/$vertex5/" system/blockMeshDict
sed -i "$((verticesLineNum+9))s/.*/$vertex6/" system/blockMeshDict
sed -i "$((verticesLineNum+10))s/.*/$vertex7/" system/blockMeshDict

#Updates refinementBox
refinementBoxLineNum="$(grep -n "refinementBox" system/snappyHexMeshDict | head -n 1 | cut -d: -f1)"
minRefine="        min ($7 $8 $9);"
maxRefine="        max (${10} ${11} ${12});"
sed -i "$((refinementBoxLineNum+3))s/.*/$minRefine/" system/snappyHexMeshDict
sed -i "$((refinementBoxLineNum+4))s/.*/$maxRefine/" system/snappyHexMeshDict

#Sets the location of the inside point
insidePointLineNum="$(grep -n "insidePoint" system/snappyHexMeshDict | head -n 1 | cut -d: -f1)"
insidePointX=$(echo "$7+0.005" | bc)
insidePointY=$(echo "$8+0.005" | bc)
insidePointZ=$(echo "$9+0.005" | bc)
insidePoint="    insidePoint ($insidePointX $insidePointY $insidePointZ);"
sed -i "$((insidePointLineNum))s/.*/$insidePoint/" system/snappyHexMeshDict

#Sets the bounds of the box enclosing the horizontal stabiliser
tailBoundsLineNum="$(grep -n "TailBounds" system/createZonesDict | head -n 1 | cut -d: -f1)"
minTailBounds="(${13} ${14} ${15})"
maxTailBounds="(${16} ${17} ${18})"


#Sets the bounds of the box that measures tail upstream velocity
tailUpstreamBoxLineNum="$(grep -n "TailUpstreamBox" system/createZonesDict | head -n 1 | cut -d: -f1)"
minUpstreamX=$(echo "${16}+0.05*(${16}-(${13}))" | bc)
maxUpstreamX=$(echo "${16}+0.2*(${16}-(${13}))" | bc)

minUpstreamBox="($minUpstreamX ${14} ${15})"
maxUpstreamBox="($maxUpstreamX ${17} ${18})"
sed -i "$((tailUpstreamBoxLineNum+1))s/.*/    box $minUpstreamBox $maxUpstreamBox;/" system/createZonesDict
