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

    %Determines point to define bezier curve
    P1 = [0,-fuselage_height*0.5];
    P2 = [-fuselage_param_1,-fuselage_height*0.5];
    P3 = [-fuselage_param_2,fuselage_height*0.5];
    P4 = [0,fuselage_height*0.5];


    num_points = 10;

    %Generates an array of values of t;
    t = linspace(0, 1, num_points)';

    B_left = (1-t).^3 * P1 + t.*(3*(1-t).^2)*P2 + (3*(1-t).*t.^2)*P3 + t.^3 * P4;
    B_right = flip([-B_left(2:end-1,1),B_left(2:end-1,2)], 1);


    upper_rows_left = B_left(:, 2) >= 0;
    upper_rows_right = B_right(:, 2) >= 0;

    lower_rows_left = B_left(:, 2) < 0;
    lower_rows_right = B_right(:, 2) < 0;

    B_upper = [B_left(upper_rows_left, :); B_right(upper_rows_right, :)];
    B_lower = [B_right(lower_rows_right, :); B_left(lower_rows_left, :)];


    output_profile = profile(B_upper, B_lower, 1, plastic_thickness, mesh_resolution);


end