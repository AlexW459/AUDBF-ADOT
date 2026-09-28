%Name:    Isaac Nakone
%Date:    11/12/2023
%Purpose: This is a function which takes in an 
%         aircraft and outputs the clearance
%         fuselage width to the minimum fuselage width

%Description from constraint set:
%   


function FW = con_FW( plane)

    plane.fuselage.min_width2 = plane.battery.width+2*plane.fuselage.object_spacing;
    
    
    %2024 competition min fuselage width is based on passenges. In 2025
    %there is no internal payload, os its based on battery width
    %plane.fuselage.min_width2 = plane.passengers.nwidth*plane.passengers.doll_width+...
    %    3*plane.fuselage.object_spacing;
    
    %plane.fuselage.min_width3 = plane.passengers.nwidth*...
    %    (plane.passengers.doll_width+plane.fuselage.object_spacing)+...
    %    plane.fuselage.object_spacing;
    
    plane.fuselage.min_width = max(plane.fuselage.min_width2,plane.fuselage.min_width3);
    
    FW = plane.fuselage.width - plane.fuselage.min_width;

   

end