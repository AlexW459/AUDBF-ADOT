# First 3 arguments are the COM
# Next 3 arguments are the gravity vector (not normalised)
# Next argument is the velocity magnitude
# Next argument is the air density
# Next argument is the number of force regions
# Next argument is the number of velocity regions
# Final argument is the case number

#Enters case
caseNum="Aerodynamics_Simulation_${11}"
cd $caseNum

#Set force coeffs values
sed -i "$((9))s/.*/    rho   $8;/" system/forces
sed -i "$((9))s/.*/    CofR            ($1 $2 $3);/" system/forces
sed -i "$((11))s/.*/    magUInf         $7;/" system/forces

#Set gravity vector
sed -i "$((9))s/.*/value           ($4 $5 $6);/" constant/g


#Clear file past line 18
sed -i '18,$d' system/forces

#Creates forces function objects
for ((i=0; i<$9; i++)); do
cat <<EOF >> aeroForces_$i
{
    type            forces;
    libs            ("libforces.so");
    patches         (forcePatch_$i);
    
    rho             $8;

    CofR            ($1 $2 $3);

    magUInf         $7;

    writeFields     yes;
    writeControl outputTime;
    writeInterval 1;
    writeFormat     ascii;
}
EOF
done

#Clear file past line 17
sed -i '17,$d' system/velocityMagnitudes


#Creates velocity magnitude function objects
for ((i=0; i<$10; i++)); do
cat <<EOF >> velocity_$i
{
    type            volFieldValue;
    libs            ("libfieldFunctionObjects.so");

    regionType      zone;
    cellZone        velZone_$1;
    operation       volAverage;

    fields          (magU);

    writeFields     yes;
    writeControl    outputTime;
    writeFormat     ascii;
    writeInterval   1;
}
EOF
done