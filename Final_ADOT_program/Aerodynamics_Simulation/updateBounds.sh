#!/bin/bash

# First 6 arguments are the 3 min bounds and 3 max bounds of the volume
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
minRefine="        min (${7} ${8} ${9});"
maxRefine="        max (${10} ${11} ${12});"
sed -i "$((refinementBoxLineNum+3))s/.*/$minRefine/" Aerodynamics_Simulation/system/snappyHexMeshDict
sed -i "$((refinementBoxLineNum+4))s/.*/$maxRefine/" Aerodynamics_Simulation/system/snappyHexMeshDict

#Sets the location of the inside point
insidePointLineNum="$(grep -n "insidePoint" Aerodynamics_Simulation/system/snappyHexMeshDict | head -n 1 | cut -d: -f1)"
insidePointX=$(echo "$7+0.001" | bc)
insidePoint="    insidePoint ($insidePointX 0.001 0.001);"
sed -i "$((insidePointLineNum))s/.*/$insidePoint/" Aerodynamics_Simulation/system/snappyHexMeshDict
