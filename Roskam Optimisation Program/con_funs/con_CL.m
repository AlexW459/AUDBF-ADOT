%Name:    Isaac Nakone
%Date:    11/12/2023
%Purpose: This is a function which takes in an 
%         aircraft and outputs the clearance
%         in the battery capacity.

%Description from constraint set:
%   Battery Capacity versus cruise required (BC).


function CL = con_CL( plane)

    
    CL = [plane.wing.coeff_lift_max - plane.cruise_properties.coeff_lift2;
        plane.wing.coeff_lift_max - plane.cruise_properties.coeff_lift3];
    
    
end