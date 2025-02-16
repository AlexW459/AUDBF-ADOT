%Name: Alex
%Date: 10/8/24
%Description: Extrudes the wing profile to produce the tip of the wing

function [wing_tip, pivot_point] = extrude_wing_tip_right(profiles, profile_names, ...
    parameters, param_names, der_params, der_param_names, density)
    wing_base_profile = profiles{find(strcmp(profile_names, "wing_profile"))};

    %Gets relevant variables
    wing_root_thickness = parameters(find(strcmp(param_names, "wing_root_thickness")));


    wing_tip_length = der_params(find(strcmp(der_param_names, "wing_tip_length")));

    segment_sweep = der_params(find(strcmp(der_param_names, "wing_tip_sweep")));

    base_segment_scale = der_params(find(strcmp(der_param_names, "wing_base_scale")));

    motor_scale = der_params(find(strcmp(der_param_names, "wing_motor_scale")));

    wing_tip_scale = der_params(find(strcmp(der_param_names, "wing_tip_scale")));


    extrude_marks = [0, wing_tip_length];

    side_profile_matrix = [0.5*wing_root_thickness*base_segment_scale*motor_scale, ...
                           -0.5*wing_root_thickness*base_segment_scale*motor_scale; ...
                           0.5*wing_root_thickness*motor_scale*base_segment_scale*wing_tip_scale, ...
                           -0.5*wing_root_thickness*motor_scale*base_segment_scale*wing_tip_scale];

    wing_base_profile.translate_direction = -1;

    wing_tip = extrusion(wing_base_profile, extrude_marks,...
                               side_profile_matrix, segment_sweep, density);

    pivot_point = [0, 0, 0];
end