
#Cleans mesh
surfaceCheck aircraftMesh/aircraftModelRaw.obj -splitNonManifold
cp aircraftModelRaw*.obj splitPatches
rm aircraftModelRaw*.obj
rm badFaces
rm aircraftMesh/zone_aircraftModelRaw.vtk

#Only retains largest mesh
largestFile=$(wc -l *splitPatches/aircraftModelRaw*.obj | sort -n | tail -n 2 | head -n 1 | awk '{print $2}')
cp $largestFile aircraftMesh/aircraftModelSplit.obj
rm splitPatches/*


surfaceLambdaMuSmooth aircraftMesh/aircraftModelSplit.obj aircraftMesh/aircraftModel.obj 0.8 0.2 30

gzip aircraftMesh/aircraftModel.obj

cp aircraftMesh/aircraftModel.obj.gz constant/geometry

rm -r 0/*
cp -a initialValues/. 0/

rm -r constant/polyMesh

rm -r constant/extendedFeatureEdgeMesh

rm constant/geometry/aircraftModel.eMesh

blockMesh
surfaceFeatures

decomposePar -copyZero

mpirun -np 4 snappyHexMesh -parallel -overwrite

find . -type f -iname "*level*" -exec rm {} \;

mpirun renumberMesh -overwrite -parallel

mpirun potentialFoam -initialiseUBCs -parallel -writep

reconstructPar -withZero -constant

rm -r  processor*


