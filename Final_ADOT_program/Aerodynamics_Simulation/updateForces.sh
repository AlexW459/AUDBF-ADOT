# First 3 arguments are the COM
# Next 3 arguments are the normalised upwards vector
# Next 3 arguments are the normalised velocity vector
# Next argument is the velocity magnitude


#Set force coeffs values
COMlineNum="$(grep -n "CofR" Aerodynamics_Simulation/system/forces | head -n 1 | cut -d: -f1)"
liftDirlineNum="$(grep -n "liftDir" Aerodynamics_Simulation/system/forces | head -n 1 | cut -d: -f1)"
dragDirlineNum="$(grep -n "dragDir" Aerodynamics_Simulation/system/forces | head -n 1 | cut -d: -f1)"
velInflineNum="$(grep -n "magUInf" Aerodynamics_Simulation/system/forces | head -n 1 | cut -d: -f1)"

sed -i "$((COMlineNum))s/.*/    CofR            ($1 $2 $3);/" Aerodynamics_Simulation/system/forces
sed -i "$((liftDirlineNum))s/.*/    liftDir         ($4 $5 $6);/" Aerodynamics_Simulation/system/forces
sed -i "$((dragDirlineNum))s/.*/    dragDir         ($7 $8 $9);/" Aerodynamics_Simulation/system/forces
sed -i "$((velInflineNum))s/.*/    magUInf         ${10};/" Aerodynamics_Simulation/system/forces