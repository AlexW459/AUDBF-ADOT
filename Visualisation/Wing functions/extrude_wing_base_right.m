%Name: Alex
%Date: 4/8/24
%Description: Extrudes the wing profile to produce the wing object


function [wing_extrusion, pivot_point] = extrude_wing_base_right(profiles, ...
    profile_names, parameters, param_names, der_params, der_param_names, density)
    %Gets profile of wing
    wing_base_profile = profiles{find(strcmp(profile_names, "wing_profile"))};
    wing_root_thickness = parameters(find(strcmp(param_names, "wing_root_thickness")));

    wing_segment_length = der_params(find(strcmp(der_param_names, "wing_base_length")));

    segment_sweep = der_params(find(strcmp(der_param_names, "wing_base_sweep")));

    segment_scale = der_params(find(strcmp(der_param_names, "wing_base_scale")));
    
    wing_extrude_marks = [0, wing_segment_length];

    wing_base_profile = wing_base_profile.change_direction(-1);

    wing_side_profile_matrix = [0.5*wing_root_thickness, ...
                                -0.5*wing_root_thickness;...
                                0.5*wing_root_thickness*segment_scale, ...
                                -0.5*wing_root_thickness*segment_scale];

    wing_extrusion = extrusion(wing_base_profile, wing_extrude_marks,...
                               wing_side_profile_matrix, segment_sweep, density);
    
    pivot_point = [0, 0, 0];

end