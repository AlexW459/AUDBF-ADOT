%Name: Alex
%Date: 4/8/24
%Description: Extrudes the wing profile to produce the wing object


function [wing_extrusion, pivot_point] = extrude_wing_right(profiles, ...
    profile_names, parameters, param_names, der_params, der_param_names, density)
    %Gets profile of wing
    wing_base_profile = profiles{find(strcmp(profile_names, "wing_profile"))};
    wing_scale = parameters(find(strcmp(param_names, "wing_scale")));
    wing_length = parameters(find(strcmp(param_names, "wing_length")));
    wing_sweep = parameters(find(strcmp(param_names, "wing_sweep")));

    wing_extrude_marks = [0, -wing_length];

    scale_vals = [1, wing_scale];

    xy_pos_vals = [0, 0; wing_sweep, 0];


    wing_extrusion = extrusion(wing_base_profile, wing_extrude_marks,...
                               xy_pos_vals, scale_vals, density);
    
    pivot_point = [0, 0, 0];

end