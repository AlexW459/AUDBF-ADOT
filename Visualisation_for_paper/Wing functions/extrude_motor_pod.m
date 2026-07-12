%Name: Alex
%Date: 15/8/24
%Description: Extrudes the motor pod profile to produce the motor pod
%object

function [motor_pod, pivot_point] = extrude_motor_pod(profiles, profile_names, ...
    parameters, param_names, der_params, der_param_names, density)
    pod_profile = profiles{find(strcmp(profile_names, "motor_pod_profile"))};

    motor_pod_length = der_params(find(strcmp(der_param_names, "motor_pod_length")));

    motor_mass = der_params(find(strcmp(der_param_names, "motor_mass")));

    z_sample = [0, -motor_pod_length];

    xy_pos_vals = [0, 0; 0, 0];

    scale_vals = [1, 1];

    motor_pod = extrusion(pod_profile, z_sample, xy_pos_vals, scale_vals, density, ...
        motor_mass, [0, 0, 0]);

    pivot_point = [0, 0, 0];


end