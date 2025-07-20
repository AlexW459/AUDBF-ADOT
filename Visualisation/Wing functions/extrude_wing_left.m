%Name: Alex
%Date: 7/8/24
%Description: Extrudes the wing profile to produce the first half of the
%wing


function [wing_extrusion, pivot_point] = extrude_wing_left(profiles, profile_names, ...
    parameters, param_names, der_params, der_param_names, density)
    %Gets profile of wing
    wing_base_profile = profiles{find(strcmp(profile_names, "wing_profile"))};
    wing_length = parameters(find(strcmp(param_names, "wing_length")));
    wing_sweep = parameters(find(strcmp(param_names, "wing_sweep")));
    wing_scale = parameters(find(strcmp(param_names, "wing_scale")));

    %Gets relevant variables
    wing_root_thickness = parameters(find(strcmp(param_names, "wing_root_thickness")));


    wing_extrude_marks = [0, wing_length];


    wing_side_profile_matrix = [0.5*wing_root_thickness, ...
                                 -0.5*wing_root_thickness;...
                                0.5*wing_root_thickness*wing_scale, ...
                                -0.5*wing_root_thickness*wing_scale];


    wing_extrusion = extrusion(wing_base_profile, wing_extrude_marks,...
                               wing_side_profile_matrix, wing_sweep, density);

    pivot_point = [0, 0, 0];


end