%Name: Alex
%Date: 2/8/24
%Description: %Function that determines the transformations that need to be
%applied to the fuselage based on the aircraft parameters

function computed_values = fuselage_constraint(parameters, param_names, der_params, der_param_names)
    xPos = 0;
    yPos = 0;
    zPos = 0;

    
    quat = quat2axang(eul2quat([-90/180*pi, -90/180*pi, 0/180*pi]));

    xQuat = -sqrt(1/3);
    yQuat = -sqrt(1/3);
    zQuat = -sqrt(1/3);
    deg = 120;

    computed_values = [xPos, yPos, zPos, xQuat, yQuat, zQuat, deg];

end