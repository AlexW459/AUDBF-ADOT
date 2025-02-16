%Name: Alex
%Date: 19/8/24
%Description: Creates the profile that can be extruded to produce the
%empenage rods

function output_profile = empennage_support_profile(parameters, param_names, der_params, der_param_names)
    support_thickness = ...
        parameters(find(strcmp(param_names, "empennage_supports_thickness")));

    plastic_thickness = parameters(find(strcmp(param_names, "plastic_thickness")));

    mesh_resolution = parameters(find(strcmp(param_names, "mesh_resolution")));

    num_points = 8;

    max_degrees = 180;

    %Generates a circle of points
    t = linspace(0, max_degrees, num_points)';

    
    B_upper = [-0.5*support_thickness*cosd(t), 0.5*support_thickness*sind(t)];
    B_lower = [0.5*support_thickness*cosd(t), -0.5*support_thickness*sind(t)];

    % plot(B_upper(:, 1), B_upper(:, 2), 'r-');

    output_profile = profile(B_upper, B_lower, -1, plastic_thickness, mesh_resolution*16);

end
