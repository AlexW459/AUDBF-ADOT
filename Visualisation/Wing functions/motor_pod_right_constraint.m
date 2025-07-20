%Name: Alex
%Date: 15/8/24
%Description: Moves the right motor pod to the correct position

function transformations = motor_pod_right_constraint(parameters, param_names, ...
    der_params, der_param_names)
    motor_width = der_params(find(strcmp(der_param_names, "motor_width")));

    wing_base_length = der_params(find(strcmp(der_param_names, "wing_base_length")));

    motor_pod_xPos = der_params(find(strcmp(der_param_names, "motor_pod_xPos")));


    xPos = motor_pod_xPos;
    yPos = 0;
    zPos = -wing_base_length-motor_width*0.5;

    


    xQuat = 0;
    yQuat = 1;
    zQuat = 0;
    deg = 90;

    transformations = [xPos, yPos, zPos, xQuat, yQuat, zQuat, deg];
end