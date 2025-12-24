#!/bin/bash

# First 6 arguments are the 3 min bounds and 3 max bounds of the volume
# Next 6 arguments are the 3 min bounds and 3 max bounds of the refinement box
# Next 6 arguments are the 3 min bounds and 3 max bounds of the horizontal stabiliser

cd Aerodynamics_Simulation

. /opt/openfoam13/etc/bashrc


#Cleans mesh
surfaceSplitByTopology aircraftMesh/aircraftModelRaw.obj splitPatches/aircraftModelSplit.obj

#Only retains largest mesh
largestFile=$(wc -l *splitPatches/aircraftModelSplit_multiplePart_*.obj | sort -n | tail -n 2 | head -n 1 | awk '{print $2}')
cp $largestFile aircraftMesh/aircraftModel.obj

rm splitPatches/*

surfaceLambdaMuSmooth aircraftMesh/aircraftModelRaw.obj aircraftMesh/aircraftModel.obj 0.5 0.5 20 
gzip aircraftMesh/aircraftModel.obj

surfaceLambdaMuSmooth aircraftMesh/horizontalStabiliserRaw.obj aircraftMesh/horizontalStabiliser.obj 0.5 0.5 20 
gzip aircraftMesh/horizontalStabiliser.obj


rm constant/geometry/aircraftModel.obj.gz
cp aircraftMesh/aircraftModel.obj.gz constant/geometry

rm constant/geometry/horizontalStabiliser.obj.gz
cp aircraftMesh/horizontalStabiliser.obj.gz constant/geometry


rm -r constant/polyMesh
rm -r 0/polyMesh

rm -r constant/extendedFeatureEdgeMesh


rm constant/geometry/aircraftModel.eMesh
rm constant/geometry/horizontalStabiliser.eMesh


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
maxUpstreamX=$(echo "${16}+0.1*(${16}-(${13}))" | bc)

minUpstreamBox="($minUpstreamX ${14} ${15})"
maxUpstreamBox="($maxUpstreamX ${17} ${18})"
sed -i "$((tailUpstreamBoxLineNum+1))s/.*/    box $minUpstreamBox $maxUpstreamBox;/" system/createZonesDict


blockMesh > blockLog
surfaceFeatures > surfaceLog

#decomposePar -copyZero

#mpirun -np 4 snappyHexMesh -parallel -overwrite
snappyHexMesh -overwrite > meshLog
createZones

#find . -type f -iname "*level*" -exec rm {} \;

#mpirun renumberMesh -constant -overwrite -parallel
