

rm -r 0/*
cp -a initialValues/. 0/

rm -r constant/polyMesh

rm -r constant/extendedFeatureEdgeMesh

rm constant/geometry/motorBike.eMesh



blockMesh
surfaceFeatures

decomposePar -copyZero

mpirun -np 4 snappyHexMesh -parallel -overwrite

find . -type f -iname "*level*" -exec rm {} \;

mpirun renumberMesh -overwrite -parallel

mpirun potentialFoam -initialiseUBCs -parallel -writep

reconstructPar -withZero -constant

rm -r  processor*


