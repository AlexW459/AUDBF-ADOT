%Name: Alex
%Date: 2/8/24
%Description: %Function that generates the fuselage profile based on aircraft parameters

function output_profile = fuselage_profile(parameters, param_names, der_params, der_param_names)
    fuselage_height = ...
        parameters(find(strcmp(param_names, "fuselage_height")));
    fuselage_param_1= ...
        parameters(find(strcmp(param_names, "fuselage_param_1")));
    fuselage_param_2 = ...
        parameters(find(strcmp(param_names, "fuselage_param_2")));
    plastic_thickness = ...
        parameters(find(strcmp(param_names, "plastic_thickness")));

    mesh_resolution= ...
        parameters(find(strcmp(param_names, "mesh_resolution")));

    %Determines points to define bezier curve
    P1 = [0,-fuselage_height*0.5];
    P2 = [-fuselage_param_1,-fuselage_height*0.5];
    P3 = [-fuselage_param_2,fuselage_height*0.5];
    P4 = [0,fuselage_height*0.5];


    num_points = ceil((fuselage_height+fuselage_param_1+fuselage_param_2)/3*pi*mesh_resolution);

    %Generates an array of values of t;
    t = linspace(0, 1, num_points)';

    B_left = (1-t).^3 * P1 + t.*(3*(1-t).^2)*P2 + (3*(1-t).*t.^2)*P3 + t.^3 * P4;
    B_right = flip([-B_left(2:end-1,1), B_left(2:end-1,2)], 1);

    B = [B_left; B_right];

    %plot(B(:, 1), B(:, 2));

    output_profile = profile(B, plastic_thickness);


end