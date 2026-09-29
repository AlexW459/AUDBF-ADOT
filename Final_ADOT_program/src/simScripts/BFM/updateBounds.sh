#!/bin/bash

# First 6 arguments are the 3 min bounds and 3 max bounds of the outer box
# Next 6 arguments are the 3 min bounds and 3 max bounds of the inner box
# Next 3 arguments are the size of the inner box
# Next 3 arguments are the number of cells on either side of the inner box
# Next argument is the grading amount
# Next argument is the grading amount inverse
# Next 3 arguments is location of point in mesh
# Next argument is the test number


#Enters case
caseNum="Aerodynamics_Simulation_BFM_Test_${24}"
cd $caseNum
cd system

#Updates bounding box
verticesLineNum="$(grep -n "vertices" blockMeshDict | head -n 1 | cut -d: -f1)"

sed -i "$((verticesLineNum+2))s/.*/   ($1 $2 $3)/" blockMeshDict
sed -i "$((verticesLineNum+3))s/.*/   ($7 $2 $3)/" blockMeshDict
sed -i "$((verticesLineNum+4))s/.*/   (${10} $2 $3)/" blockMeshDict
sed -i "$((verticesLineNum+5))s/.*/   ($4 $2 $3)/" blockMeshDict

sed -i "$((verticesLineNum+6))s/.*/   ($1 $8 $3)/" blockMeshDict
sed -i "$((verticesLineNum+7))s/.*/   ($7 $8 $3)/" blockMeshDict
sed -i "$((verticesLineNum+8))s/.*/   (${10} $8 $3)/" blockMeshDict
sed -i "$((verticesLineNum+9))s/.*/   ($4 $8 $3)/" blockMeshDict

sed -i "$((verticesLineNum+10))s/.*/   ($1 ${11} $3)/" blockMeshDict
sed -i "$((verticesLineNum+11))s/.*/   ($7 ${11} $3)/" blockMeshDict
sed -i "$((verticesLineNum+12))s/.*/   (${10} ${11} $3)/" blockMeshDict
sed -i "$((verticesLineNum+13))s/.*/   ($4 ${11} $3)/" blockMeshDict

sed -i "$((verticesLineNum+14))s/.*/   ($1 $5 $3)/" blockMeshDict
sed -i "$((verticesLineNum+15))s/.*/   ($7 $5 $3)/" blockMeshDict
sed -i "$((verticesLineNum+16))s/.*/   (${10} $5 $3)/" blockMeshDict
sed -i "$((verticesLineNum+17))s/.*/   ($4 $5 $3)/" blockMeshDict


sed -i "$((verticesLineNum+18))s/.*/   ($1 $2 $9)/" blockMeshDict
sed -i "$((verticesLineNum+19))s/.*/   ($7 $2 $9)/" blockMeshDict
sed -i "$((verticesLineNum+20))s/.*/   (${10} $2 $9)/" blockMeshDict
sed -i "$((verticesLineNum+21))s/.*/   ($4 $2 $9)/" blockMeshDict

sed -i "$((verticesLineNum+22))s/.*/   ($1 $8 $9)/" blockMeshDict
sed -i "$((verticesLineNum+23))s/.*/   ($7 $8 $9)/" blockMeshDict
sed -i "$((verticesLineNum+24))s/.*/   (${10} $8 $9)/" blockMeshDict
sed -i "$((verticesLineNum+25))s/.*/   ($4 $8 $9)/" blockMeshDict

sed -i "$((verticesLineNum+26))s/.*/   ($1 ${11} $9)/" blockMeshDict
sed -i "$((verticesLineNum+27))s/.*/   ($7 ${11} $9)/" blockMeshDict
sed -i "$((verticesLineNum+28))s/.*/   (${10} ${11} $9)/" blockMeshDict
sed -i "$((verticesLineNum+29))s/.*/   ($4 ${11} $9)/" blockMeshDict

sed -i "$((verticesLineNum+30))s/.*/   ($1 $5 $9)/" blockMeshDict
sed -i "$((verticesLineNum+31))s/.*/   ($7 $5 $9)/" blockMeshDict
sed -i "$((verticesLineNum+32))s/.*/   (${10} $5 $9)/" blockMeshDict
sed -i "$((verticesLineNum+33))s/.*/   ($4 $5 $9)/" blockMeshDict


sed -i "$((verticesLineNum+34))s/.*/   ($1 $2 ${12})/" blockMeshDict
sed -i "$((verticesLineNum+35))s/.*/   ($7 $2 ${12})/" blockMeshDict
sed -i "$((verticesLineNum+36))s/.*/   (${10} $2 ${12})/" blockMeshDict
sed -i "$((verticesLineNum+37))s/.*/   ($4 $2 ${12})/" blockMeshDict

sed -i "$((verticesLineNum+38))s/.*/   ($1 $8 ${12})/" blockMeshDict
sed -i "$((verticesLineNum+39))s/.*/   ($7 $8 ${12})/" blockMeshDict
sed -i "$((verticesLineNum+40))s/.*/   (${10} $8 ${12})/" blockMeshDict
sed -i "$((verticesLineNum+41))s/.*/   ($4 $8 ${12})/" blockMeshDict

sed -i "$((verticesLineNum+42))s/.*/   ($1 ${11} ${12})/" blockMeshDict
sed -i "$((verticesLineNum+43))s/.*/   ($7 ${11} ${12})/" blockMeshDict
sed -i "$((verticesLineNum+44))s/.*/   (${10} ${11} ${12})/" blockMeshDict
sed -i "$((verticesLineNum+45))s/.*/   ($4 ${11} ${12})/" blockMeshDict

sed -i "$((verticesLineNum+46))s/.*/   ($1 $5 ${12})/" blockMeshDict
sed -i "$((verticesLineNum+47))s/.*/   ($7 $5 ${12})/" blockMeshDict
sed -i "$((verticesLineNum+48))s/.*/   (${10} $5 ${12})/" blockMeshDict
sed -i "$((verticesLineNum+49))s/.*/   ($4 $5 ${12})/" blockMeshDict


sed -i "$((verticesLineNum+50))s/.*/   ($1 $2 $6)/" blockMeshDict
sed -i "$((verticesLineNum+51))s/.*/   ($7 $2 $6)/" blockMeshDict
sed -i "$((verticesLineNum+52))s/.*/   (${10} $2 $6)/" blockMeshDict
sed -i "$((verticesLineNum+53))s/.*/   ($4 $2 $6)/" blockMeshDict

sed -i "$((verticesLineNum+54))s/.*/   ($1 $8 $6)/" blockMeshDict
sed -i "$((verticesLineNum+55))s/.*/   ($7 $8 $6)/" blockMeshDict
sed -i "$((verticesLineNum+56))s/.*/   (${10} $8 $6)/" blockMeshDict
sed -i "$((verticesLineNum+57))s/.*/   ($4 $8 $6)/" blockMeshDict

sed -i "$((verticesLineNum+58))s/.*/   ($1 ${11} $6)/" blockMeshDict
sed -i "$((verticesLineNum+59))s/.*/   ($7 ${11} $6)/" blockMeshDict
sed -i "$((verticesLineNum+60))s/.*/   (${10} ${11} $6)/" blockMeshDict
sed -i "$((verticesLineNum+61))s/.*/   ($4 ${11} $6)/" blockMeshDict

sed -i "$((verticesLineNum+62))s/.*/   ($1 $5 $6)/" blockMeshDict
sed -i "$((verticesLineNum+63))s/.*/   ($7 $5 $6)/" blockMeshDict
sed -i "$((verticesLineNum+64))s/.*/   (${10} $5 $6)/" blockMeshDict
sed -i "$((verticesLineNum+65))s/.*/   ($4 $5 $6)/" blockMeshDict


#Sets blockmesh size
boundx=${16}
boundy=${17}
boundz=${18}
gradingAmount=${19} 
gradingInv=${20}
hexLineNum="$(grep -n "blocks" blockMeshDict | head -n 1 | cut -d: -f1)"

sed -i "$((hexLineNum+2))s/.*/    hex (0 1 5 4 16 17 21 20) ($boundx $boundy $boundz) simpleGrading ($gradingInv $gradingInv $gradingInv)/" blockMeshDict
sed -i "$((hexLineNum+3))s/.*/    hex (1 2 6 5 17 18 22 21) (${13} $boundy $boundz) simpleGrading (1 $gradingInv $gradingInv)/" blockMeshDict
sed -i "$((hexLineNum+4))s/.*/    hex (2 3 7 6 18 19 23 22) ($boundx $boundy $boundz) simpleGrading ($gradingAmount $gradingInv $gradingInv)/" blockMeshDict

sed -i "$((hexLineNum+5))s/.*/    hex (4 5 9 8 20 21 25 24) ($boundx ${14} $boundz) simpleGrading ($gradingInv 1 $gradingInv)/" blockMeshDict
sed -i "$((hexLineNum+6))s/.*/    hex (5 6 10 9 21 22 26 25) (${13} ${14} $boundz) simpleGrading (1 1 $gradingInv)/" blockMeshDict
sed -i "$((hexLineNum+7))s/.*/    hex (6 7 11 10 22 23 27 26) ($boundx ${14} $boundz) simpleGrading ($gradingAmount 1 $gradingInv)/" blockMeshDict

sed -i "$((hexLineNum+8))s/.*/    hex (8 9 13 12 24 25 29 28) ($boundx $boundy $boundz) simpleGrading ($gradingInv $gradingAmount $gradingInv)/" blockMeshDict
sed -i "$((hexLineNum+9))s/.*/    hex (9 10 14 13 25 26 30 29) (${13} $boundy $boundz) simpleGrading (1 $gradingAmount $gradingInv)/" blockMeshDict
sed -i "$((hexLineNum+10))s/.*/    hex (10 11 15 14 26 27 31 30) ($boundx $boundy $boundz) simpleGrading ($gradingAmount $gradingAmount $gradingInv)/" blockMeshDict


sed -i "$((hexLineNum+12))s/.*/    hex (16 17 21 20 32 33 37 36) ($boundx $boundy ${15}) simpleGrading ($gradingInv $gradingInv 1)/" blockMeshDict
sed -i "$((hexLineNum+13))s/.*/    hex (17 18 22 21 33 34 38 37) (${13} $boundy ${15}) simpleGrading (1 $gradingInv 1)/" blockMeshDict
sed -i "$((hexLineNum+14))s/.*/    hex (18 19 23 22 34 35 39 38) ($boundx $boundy ${15}) simpleGrading ($gradingAmount $gradingInv 1)/" blockMeshDict

sed -i "$((hexLineNum+15))s/.*/    hex (20 21 25 24 36 37 41 40) ($boundx ${14} ${15}) simpleGrading ($gradingInv 1 1)/" blockMeshDict
sed -i "$((hexLineNum+16))s/.*/    hex (21 22 26 25 37 38 42 41) (${13} ${14} ${15}) simpleGrading (1 1 1)/" blockMeshDict
sed -i "$((hexLineNum+17))s/.*/    hex (22 23 27 26 38 39 43 42) ($boundx ${14} ${15}) simpleGrading ($gradingAmount 1 1)/" blockMeshDict

sed -i "$((hexLineNum+18))s/.*/    hex (24 25 29 28 40 41 45 44) ($boundx $boundy ${15}) simpleGrading ($gradingInv $gradingAmount 1)/" blockMeshDict
sed -i "$((hexLineNum+19))s/.*/    hex (25 26 30 29 41 42 46 45) (${13} $boundy ${15}) simpleGrading (1 $gradingAmount 1)/" blockMeshDict
sed -i "$((hexLineNum+20))s/.*/    hex (26 27 31 30 42 43 47 46) ($boundx $boundy ${15}) simpleGrading ($gradingAmount $gradingAmount 1)/" blockMeshDict


sed -i "$((hexLineNum+22))s/.*/    hex (32 33 37 36 48 49 53 52) ($boundx $boundy $boundz) simpleGrading ($gradingInv $gradingInv $gradingAmount)/" blockMeshDict
sed -i "$((hexLineNum+23))s/.*/    hex (33 34 38 37 49 50 54 53) (${13} $boundy $boundz) simpleGrading (1 $gradingInv $gradingAmount)/" blockMeshDict
sed -i "$((hexLineNum+24))s/.*/    hex (34 35 39 38 50 51 55 54) ($boundx $boundy $boundz) simpleGrading ($gradingAmount $gradingInv $gradingAmount)/" blockMeshDict

sed -i "$((hexLineNum+25))s/.*/    hex (36 37 41 40 52 53 57 56) ($boundx ${14} $boundz) simpleGrading ($gradingInv 1 $gradingAmount)/" blockMeshDict
sed -i "$((hexLineNum+26))s/.*/    hex (37 38 42 41 53 54 58 57) (${13} ${14} $boundz) simpleGrading (1 1 $gradingAmount)/" blockMeshDict
sed -i "$((hexLineNum+27))s/.*/    hex (38 39 43 42 54 55 59 58) ($boundx ${14} $boundz) simpleGrading ($gradingAmount 1 $gradingAmount)/" blockMeshDict

sed -i "$((hexLineNum+28))s/.*/    hex (40 41 45 44 56 57 61 60) ($boundx $boundy $boundz) simpleGrading ($gradingInv $gradingAmount $gradingAmount)/" blockMeshDict
sed -i "$((hexLineNum+29))s/.*/    hex (41 42 46 45 57 58 62 61) (${13} $boundy $boundz) simpleGrading (1 $gradingAmount $gradingAmount)/" blockMeshDict
sed -i "$((hexLineNum+30))s/.*/    hex (42 43 47 46 58 59 63 62) ($boundx $boundy $boundz) simpleGrading ($gradingAmount $gradingAmount $gradingAmount)/" blockMeshDict

#Updates refinementBox
refinementBoxLineNum="$(grep -n "refinementBox" snappyHexMeshDict | head -n 1 | cut -d: -f1)"
minRefine="        min ($7 $8 $9);"
maxRefine="        max (${10} ${11} ${12});"
sed -i "$((refinementBoxLineNum+3))s/.*/$minRefine/" snappyHexMeshDict
sed -i "$((refinementBoxLineNum+4))s/.*/$maxRefine/" snappyHexMeshDict

#Sets the location of the inside point
sed -i "/locationInMesh/c\    locationInMesh (${21} ${22} ${23});" snappyHexMeshDict

# Clears createZones dict beyond line 24
sed -i '25,$d' createZonesDict

# Clears createPatchDict beyond line 19
sed -i '20,$d' createPatchDict
