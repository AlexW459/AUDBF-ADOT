#!/bin/bash

# First 6 arguments are the 3 min bounds and 3 max bounds of the volume
# Next 6 arguments are the 3 min bounds and 3 max bounds of the refinement box
# Next argument is the case number
# Next argument is the number of force regions
# Next arguments are bounds of force regions, 3 min and 3 max for each
# Next argument is the number of velocity regions
# Next arguments are bounds of velocity regions, 3 min and 3 max for each

#Enters case
caseNum="Aerodynamics_Simulation_$3"
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
inMeshPointX=$(echo "$7+0.005" | bc)
inMeshPointY=$(echo "$8+0.005" | bc)
inMeshPointZ=$(echo "$9+0.005" | bc)
sed -i "/locationInMesh/c\    locationInMesh ($inMeshPointX $inMeshPointY $inMeshPointZ);" snappyHexMeshDict

# Clears createZones dict beyond line 25
sed -i '25,$d' createZonesDict

# Adds region to isolate forces on object
for ((i=0; i<$4; i++)); do
paramNum=$(($i*6+5));
cat <<EOF >> createZonesDict
Intersection{
    name forceFaceZone_$(($i))
    aircraftModelZone;
    forceZone_$i{
        zoneType face;
        type box;
        box (${!$paramNum} ${!$paramNum+1} ${!$paramNum+2}) (${!$paramNum+3} ${!$paramNum+4} ${!$paramNum+5});
    };
}
EOF
done

# Deletes all lines following line 19
sed -i '20,$d' createPatchDict


# Creates patches from force region face zones
for ((i=0; i<$4; i++)); do
cat <<EOF >> createPatchDict
    forcePatch_$i{
        patchInfo{
            type wall;
        }
        constructFrom zone;
        zone forceFaceZone_$i;
    }
}
EOF
done


# Adds regions to isolate velocity
velVarNum=$((5+$4*6));
nVelZones=${!$velVarNum};
for ((i=0; i<nVelZones; i++)); do
paramNum=$(($velVarNum+$i*6+1));
cat <<EOF >> createZonesDict
box{
    name velZone_$i
    zoneType cell;
    box (${!$paramNum} ${!$(($paramNum+1))} ${!$(($paramNum+1))}) 
        (${!$(($paramNum+3))} ${!$(($paramNum+4))} ${!$(($paramNum+5))});
};
EOF

done