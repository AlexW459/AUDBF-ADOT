%Name: Alex
%Date: 19/8/24
%Description: Function that determines the transformations that will be
%applied to the left vertical stabiliser

function transformations = vertical_stabiliser_constraint(parameters, param_names, der_params, der_param_names)
    

    vertical_stabiliser_chord = der_params(find(strcmp(der_param_names, "vertical_stabiliser_chord")));
    empennage_boom_length = der_params(find(strcmp(der_param_names, "empennage_boom_length")));

    xPos = 0;
    yPos = 0;
    zPos = -empennage_boom_length-0.5*vertical_stabiliser_chord;;

    xQuat = sqrt(1/3);
    yQuat = -sqrt(1/3);
    zQuat = -sqrt(1/3);
    deg = 120;

    transformations = [xPos, yPos, zPos, xQuat, yQuat, zQuat, deg];
end