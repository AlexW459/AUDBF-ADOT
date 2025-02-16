%Name: Alex
%Date: 10/8/24
%Description: Function that determines the transformations that will be
%applied to the left wing tip

function transformations = wing_tip_right_constraint(parameters, param_names, ...
    der_params, der_param_names)
    
    motor_width = der_params(find(strcmp(der_param_names, "motor_width")));
      
    motor_sweep = der_params(find(strcmp(der_param_names, "wing_motor_sweep")));

    base_segment_scale = der_params(find(strcmp(der_param_names, "wing_base_scale")));
    
    wing_root_chord = der_params(find(strcmp(der_param_names, "wing_root_chord")));
    
    motor_scale = der_params(find(strcmp(der_param_names, "wing_motor_scale")));


    xPos = -motor_width*0.5;
    yPos = 0;%-wing_vert_pos*base_segment_scale;
    zPos = -motor_sweep-0.5*wing_root_chord*base_segment_scale*motor_scale;

    xQuat = 0;
    yQuat = 1;
    zQuat = 0;
    deg = -90;

    transformations = [xPos, yPos, zPos, xQuat, yQuat, zQuat, deg];

end