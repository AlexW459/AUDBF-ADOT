%Name: Alex
%Date: 15/8/24
%Description: Extrudes the motor pod profile to produce the motor pod
%object

function [motor_pod, pivot_point] = extrude_motor_pod(profiles, profile_names, ...
    parameters, param_names, der_params, der_param_names, density)
    pod_profile = profiles{find(strcmp(profile_names, "motor_pod_profile"))};

    pod_profile = pod_profile.change_direction(-1);

    motor_width = der_params(find(strcmp(der_param_names, "motor_width")));

    plastic_thickness = parameters(find(strcmp(param_names, "plastic_thickness")));
    
    pod_width = motor_width+2*plastic_thickness;

    motor_pod_length = der_params(find(strcmp(der_param_names, "motor_pod_length")));

    motor_length = der_params(find(strcmp(der_param_names, "motor_length")));

    motor_mass = der_params(find(strcmp(der_param_names, "motor_mass")));


    x_sample = [0, motor_pod_length];

    side_profile = [0.5*1.5*pod_width, -0.5*1.5*pod_width;
                    0.5*1.5*pod_width, -0.5*1.5*pod_width];

    motor_pod = extrusion(pod_profile, x_sample, side_profile, 0, density, ...
        motor_mass, [-0.5*motor_length, 0, 0]);

    pivot_point = [0, 0, 0];

end