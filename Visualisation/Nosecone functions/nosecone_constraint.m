%Name: Alex
%Date: 2/8/24
%Description: Function that calculates the transformations that should be
%applied to the base (hollow section) of the nosecone based on the aircraft
%parameters


function computed_values = nosecone_constraint(parameters, param_names, der_params, der_param_names)
    fuselage_length = ...
        parameters(find(strcmp(param_names, "fuselage_length")));


    xPos = 0;
    yPos = 0;
    zPos = fuselage_length-0.005;

    xQuat = 1;
    yQuat = 0;
    zQuat = 0;
    deg = 0;

    computed_values = [xPos, yPos, zPos, xQuat, yQuat, zQuat, deg];
end
