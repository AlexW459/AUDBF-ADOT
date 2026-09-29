#!/bin/bash

# First argument is the test number
# Second argument is the force region number
# Next 6 arguments are the bounds of the force region

#Enters case
caseDir="Aerodynamics_Simulation_BFM_Test_$1"
cd $caseDir/system

# Creates face zone from intersection of box and model surface
cat >> createZonesDict<<EOF
Intersection{
    name forceFaceZone_$2;
    testModelZone;
    forceZone_$2{
        zoneType face;
        type box;
        box ($3 $4 $5 $6 $7 $8);
    };
}
EOF

# Creates patch from force region face zone
cat >> createPatchDict<<EOF
    forcePatch_$2{
        patchInfo{
            type wall;
        }
        constructFrom zone;
        zone forceFaceZone_$2;
    };
EOF

