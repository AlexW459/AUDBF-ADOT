%Name: Alex
%Date: 19/8/24
%Description: Extrudes the 


function [horizontal_stabiliser, pivot_point] = extrude_horizontal_stabiliser(profiles, ...
    profile_names, parameters, param_names, der_params, der_param_names, density)

    horizontal_stabiliser_profile = profiles{find(strcmp(profile_names, "horizontal_stabiliser_profile"))};
    horizontal_stabiliser_width = der_params(find(strcmp(der_param_names, "horizontal_stabiliser_width")));

    scale_vals = [1, 1];
    xy_pos_vals = [0, 0; 0, 0];
    extrude_marks = [-0.5*horizontal_stabiliser_width, horizontal_stabiliser_width*0.5];

    horizontal_stabiliser = extrusion(horizontal_stabiliser_profile, extrude_marks,...
                               xy_pos_vals, scale_vals, density);

    pivot_point = [0, 0, 0];
end