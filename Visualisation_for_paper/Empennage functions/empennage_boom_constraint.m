%Name: Alex
%Date: 2/8/24
%Description: Function that determines the transformations that will be
%applied to both the left and right empennage supports

function transformations = empennage_boom_constraint(parameters, param_names, der_params, der_param_names)
    motor_pod_length = der_params(find(strcmp(der_param_names, "motor_pod_length")));

    wing_root_chord = der_params(find(strcmp(der_param_names, "wing_root_chord")));

    xPos = 0;
    yPos = 0;
    zPos = -motor_pod_length;

    xQuat = 1;
    yQuat = 0;
    zQuat = 0;
    deg = 0;


    transformations = [xPos, yPos, zPos, xQuat, yQuat, zQuat, deg];

end