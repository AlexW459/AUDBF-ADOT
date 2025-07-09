%Name: Alex
%Date: 22/8/24
%Description: Extrudes the 


function [stabiliser_extrusion, pivot_point] = extrude_vertical_stabiliser(profiles, ...
    profile_names, parameters, param_names, der_params, der_param_names, density)
    stabiliser_profile = profiles{find(strcmp(profile_names, "vertical_stabiliser_profile"))};

    vertical_stabiliser_thickness = parameters(find(strcmp(param_names, "vertical_stabiliser_thickness")));
    stabiliser_height = parameters(find(strcmp(param_names, "vertical_stabiliser_height")));

    extrude_marks = [0, stabiliser_height];

    side_profile = [0.5*vertical_stabiliser_thickness,...
        -0.5*vertical_stabiliser_thickness; 
        0.5*vertical_stabiliser_thickness,...
        -0.5*vertical_stabiliser_thickness];


    stabiliser_extrusion = extrusion(stabiliser_profile, extrude_marks,...
                               side_profile, 0, density);
    
    pivot_point = [0, 0, 0];

end