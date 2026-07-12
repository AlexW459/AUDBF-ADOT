%Name: Alex
%Date: 22/8/24
%Description: Extrudes the 


function [stabiliser_extrusion, pivot_point] = extrude_vertical_stabiliser(profiles, ...
    profile_names, parameters, param_names, der_params, der_param_names, density)
    stabiliser_profile = profiles{find(strcmp(profile_names, "vertical_stabiliser_profile"))};

    stabiliser_height = parameters(find(strcmp(param_names, "vertical_stabiliser_height")));

    extrude_marks = [0, stabiliser_height];

    scale_vals = [1, 1];
    xy_pos_vals = [0, 0; 0, 0];

    stabiliser_extrusion = extrusion(stabiliser_profile, extrude_marks,...
                               xy_pos_vals, scale_vals, density);
    
    pivot_point = [0, 0, 0];

end