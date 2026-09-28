#!/bin/bash

# First argument is the case number
# Second argument is the test number
# Third argument is the location of the OpenFOAM sourcing script

openfoamSource=$3

. $openfoamSource

#Enters case
caseNum="Aerodynamics_Simulation_BFM_${1}_$2"
cd $caseNum

cat <<EOF >> system/createPatchDict
}
EOF


#Cleans mesh
surfaceSplitByTopology testModelMesh/testModelRaw.obj splitPatches/testModelSplit.obj > splitLog

#Only retains largest mesh
largestMeshFile=$(wc -l *splitPatches/testModelSplit_*.obj | sort -n | tail -n 2 | head -n 1 | awk '{print $2}')
cp $largestMeshFile testModelMesh/testModelSplit.obj
rm splitPatches/testModel*

surfaceLambdaMuSmooth testModelMesh/testModelSplit.obj testModelMesh/testModel.obj 0.5 0.5 20 > smoothLog
gzip testModelMesh/testModel.obj


rm -f constant/geometry/aircraftModel*
cp testModelMesh/testModel.obj.gz constant/geometry/


rm -r -f constant/polyMesh/*
rm -r -f 0/*

rm -r -f constant/extendedFeatureEdgeMesh/*


blockMesh > blockLog
surfaceFeatures > surfaceLog

echo "Running snappyHexMesh on rank $1"
snappyHexMesh -overwrite > meshLog

createZones > zoneLog

createPatch > patchLog

renumberMesh -constant > renumberLog

#Clean directories
rm -r -f processor*
