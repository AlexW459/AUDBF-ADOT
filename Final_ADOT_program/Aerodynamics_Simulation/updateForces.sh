# First 3 arguments are the COM
# Next 3 arguments are the normalised upwards vector
# Next 3 arguments are the normalised velocity vector
# Next argument is the velocity magnitude


#Set force coeffs values
COMlineNum="$(grep -n "COM1" Aerodynamics_Simulation/system/forces | head -n 1 | cut -d: -f1)"
liftDirlineNum="$(grep -n "liftDir" Aerodynamics_Simulation/system/forces | head -n 1 | cut -d: -f1)"
dragDirlineNum="$(grep -n "dragDir" Aerodynamics_Simulation/system/forces | head -n 1 | cut -d: -f1)"
velInflineNum="$(grep -n "Velocity magnitude1" Aerodynamics_Simulation/system/forces | head -n 1 | cut -d: -f1)"

sed -i "$((COMlineNum+1))s/.*/    CofR            ($1 $2 $3);/" Aerodynamics_Simulation/system/forces
sed -i "$((liftDirlineNum))s/.*/    liftDir         ($4 $5 $6);/" Aerodynamics_Simulation/system/forces
sed -i "$((dragDirlineNum))s/.*/    dragDir         ($7 $8 $9);/" Aerodynamics_Simulation/system/forces
sed -i "$((velInflineNum+1))s/.*/    magUInf         ${10};/" Aerodynamics_Simulation/system/forces

tailCOMlineNum="$(grep -n "COM2" Aerodynamics_Simulation/system/forces | head -n 1 | cut -d: -f1)"
tailVelInflineNum="$(grep -n "Velocity magnitude2" Aerodynamics_Simulation/system/forces | head -n 1 | cut -d: -f1)"

sed -i "$((tailCOMlineNum+1))s/.*/    CofR            ($1 $2 $3);/" Aerodynamics_Simulation/system/forces
sed -i "$((tailVelInflineNum+1))s/.*/    magUInf         ${10};/" Aerodynamics_Simulation/system/forces
