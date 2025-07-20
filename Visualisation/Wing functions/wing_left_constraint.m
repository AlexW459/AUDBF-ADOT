%Name: Alex
%Date: 2/8/24
%Description: Function that determines the transformations that will be
%applied to the left wing

function transformations = wing_left_constraint(parameters, param_names, ...
    der_params, der_param_names)

    wing_xPos = der_params(find(strcmp(der_param_names, "wing_xPos")));
    wing_yPos = der_params(find(strcmp(der_param_names, "wing_yPos")));
    wing_zPos = der_params(find(strcmp(der_param_names, "wing_zPos")));


    xPos = wing_yPos;
    yPos = wing_zPos;
    zPos = wing_xPos;

    xQuat = 0;
    yQuat = 1;
    zQuat = 0;
    deg = -90;

    transformations = [xPos, yPos, zPos, xQuat, yQuat, zQuat, deg];

    
end