%Name: Alex
%Date: 8/8/24
%Description: Function that determines the transformations that will be
%applied to the tail

function transformations = tailcone_constraint(parameters, param_names, der_params, der_param_names)

    xPos = 0;
    yPos = 0;
    zPos = 0.01;

    xQuat = 1;
    yQuat = 0;
    zQuat = 0;
    deg = 0;

    transformations = [xPos, yPos, zPos, xQuat, yQuat, zQuat, deg];

end