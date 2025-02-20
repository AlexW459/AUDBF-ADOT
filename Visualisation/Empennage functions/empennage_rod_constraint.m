%Name: Alex
%Date: 2/8/24
%Description: Function that determines the transformations that will be
%applied to both the left and right empennage rods

function transformations = empennage_rod_constraint(parameters, param_names, der_params, der_param_names)
    support_length = der_params(find(strcmp(der_param_names, "empennage_support_length")));

    xPos = 0;
    yPos = 0;
    zPos = -support_length;

    xQuat = 1;
    yQuat = 0;
    zQuat = 0;
    deg = 0;


    transformations = [xPos, yPos, zPos, xQuat, yQuat, zQuat, deg];

end