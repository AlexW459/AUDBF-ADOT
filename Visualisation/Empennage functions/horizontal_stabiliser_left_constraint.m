%Name: Alex
%Date: 19/8/24
%Description: Function that determines the transformations that will be
%applied to both the left and right empennage supports

function transformations = horizontal_stabiliser_left_constraint(parameters,...
    param_names, der_params, der_param_names)


    horizontal_stabiliser_chord = der_params(find(strcmp(der_param_names, "horizontal_stabiliser_chord")));

    xPos = -0.005;
    yPos = 0;
    zPos = -0.5*horizontal_stabiliser_chord;

    xQuat = 0;
    yQuat = 1;
    zQuat = 0;
    deg = -90;

    transformations = [xPos, yPos, zPos, xQuat, yQuat, zQuat, deg];

end