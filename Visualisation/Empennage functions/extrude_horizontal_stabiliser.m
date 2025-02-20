%Name: Alex
%Date: 19/8/24
%Description: Extrudes the 


function [stabiliser_left, pivot_point] = extrude_horizontal_stabiliser(profiles, ...
    profile_names, parameters, param_names, der_params, der_param_names, density)

    horizontal_stabiliser_profile = profiles{find(strcmp(profile_names, "horizontal_stabiliser_profile"))};

    horizontal_stabiliser_thickness = parameters(find(strcmp(param_names, "horizontal_stabiliser_thickness")));
    horizontal_stabiliser_full_width = der_params(find(strcmp(der_param_names, "horizontal_stabiliser_full_width")));



    extrude_marks = [0, (horizontal_stabiliser_full_width)*0.5];

    side_profile = [0.5*horizontal_stabiliser_thickness, ...
        -0.5*horizontal_stabiliser_thickness;
        0.5*horizontal_stabiliser_thickness, ...
        -0.5*horizontal_stabiliser_thickness];


    horizontal_stabiliser_profile.translate_direction = -1;

    stabiliser_left = extrusion(horizontal_stabiliser_profile, extrude_marks,...
                               side_profile, 0, density);

    pivot_point = [0, 0, 0];
end