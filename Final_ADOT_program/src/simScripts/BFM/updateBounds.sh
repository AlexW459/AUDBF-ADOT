#!/bin/bash

# First 6 arguments are the 3 min bounds and 3 max bounds of the volume
# Next 6 arguments are the 3 min bounds and 3 max bounds of the refinement box
# Next 3 arguments is location of point in mesh
# Next argument is the pos number
# Next argument is the test number

#Enters case
caseNum="Aerodynamics_Simulation_BFM_${16}_${17}"
cd $caseNum
cd system

#Updates bounding box
verticesLineNum="$(grep -n "vertices" blockMeshDict | head -n 1 | cut -d: -f1)"

vertex0="   ($1 $2 $3)"
vertex1="   ($4 $2 $3)"
vertex2="   ($4 $5 $3)"
vertex3="   ($1 $5 $3)"
vertex4="   ($1 $2 $6)"
vertex5="   ($4 $2 $6)"
vertex6="   ($4 $5 $6)"
vertex7="   ($1 $5 $6)"

sed -i "$((verticesLineNum+2))s/.*/$vertex0/" blockMeshDict
sed -i "$((verticesLineNum+3))s/.*/$vertex1/" blockMeshDict
sed -i "$((verticesLineNum+4))s/.*/$vertex2/" blockMeshDict
sed -i "$((verticesLineNum+5))s/.*/$vertex3/" blockMeshDict
sed -i "$((verticesLineNum+7))s/.*/$vertex4/" blockMeshDict
sed -i "$((verticesLineNum+8))s/.*/$vertex5/" blockMeshDict
sed -i "$((verticesLineNum+9))s/.*/$vertex6/" blockMeshDict
sed -i "$((verticesLineNum+10))s/.*/$vertex7/" blockMeshDict

#Updates refinementBox
refinementBoxLineNum="$(grep -n "refinementBox" snappyHexMeshDict | head -n 1 | cut -d: -f1)"
minRefine="        min ($7 $8 $9);"
maxRefine="        max (${10} ${11} ${12});"
sed -i "$((refinementBoxLineNum+3))s/.*/$minRefine/" snappyHexMeshDict
sed -i "$((refinementBoxLineNum+4))s/.*/$maxRefine/" snappyHexMeshDict

#Sets the location of the inside point
sed -i "/locationInMesh/c\    locationInMesh (${13} ${14} ${15});" snappyHexMeshDict

# Clears createZones dict beyond line 24
sed -i '25,$d' createZonesDict

# Clears createPatchDict beyond line 19
sed -i '20,$d' createPatchDict
