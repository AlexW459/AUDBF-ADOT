%Name:        Isaac Nakone, Denis Vasilyev, Harry Rowton, Jingya Liu 
%Date:        10/12/2023
%Description: This finds the power at cruise.
%Inputs:
%       (1) V_CR  == the cruise velocity.


function [C_L_CR_2, C_L_CR_3] = C_LAtCruise(V_CR_2,V_CR_3, plane)
    C_L_CR_2 = 2*plane.mass2*plane.physics.gravity/...
             plane.physics.air_density30/V_CR_2^2/plane.wing.area;
    C_L_CR_3 = 2*plane.mass3*plane.physics.gravity/...
             plane.physics.air_density30/V_CR_3^2/plane.wing.area;
end
