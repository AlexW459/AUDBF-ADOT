# First 3 arguments are the COM
# Next 3 arguments are the normalised upwards vector
# Next 3 arguments are the normalised velocity vector
# Next argument is the velocity magnitude
# Final argument is the case number

#Enters case
caseNum="Aerodynamics_Simulation_${11}"
cd $caseNum

#Set force coeffs values
COMlineNum="$(grep -n "COM1" system/forces | head -n 1 | cut -d: -f1)"
liftDirlineNum="$(grep -n "liftDir" system/forces | head -n 1 | cut -d: -f1)"
dragDirlineNum="$(grep -n "dragDir" system/forces | head -n 1 | cut -d: -f1)"
velInflineNum="$(grep -n "Velocity magnitude1" system/forces | head -n 1 | cut -d: -f1)"

sed -i "$((COMlineNum+1))s/.*/    CofR            ($1 $2 $3);/" system/forces
sed -i "$((liftDirlineNum))s/.*/    liftDir         ($4 $5 $6);/" system/forces
sed -i "$((dragDirlineNum))s/.*/    dragDir         ($7 $8 $9);/" system/forces
sed -i "$((velInflineNum+1))s/.*/    magUInf         ${10};/" system/forces

tailCOMlineNum="$(grep -n "COM2" system/forces | head -n 1 | cut -d: -f1)"
tailVelInflineNum="$(grep -n "Velocity magnitude2" system/forces | head -n 1 | cut -d: -f1)"

sed -i "$((tailCOMlineNum+1))s/.*/    CofR            ($1 $2 $3);/" system/forces
sed -i "$((tailVelInflineNum+1))s/.*/    magUInf         ${10};/" system/forces
