%Name: Alex
%Date: 19/8/24
%Description: Generates the profile of the horizontal stabiliser on the
%empennage

function airfoil_profile = horizontal_stabiliser_profile(parameters, param_names, ...
    der_params, der_param_names)

    stabiliser_airfoil_max_camber = ...
        der_params(find(strcmp(der_param_names, "horizontal_stabiliser_airfoil_max_camber")));
    stabiliser_airfoil_max_camber_pos = ...
        der_params(find(strcmp(der_param_names, "horizontal_stabiliser_airfoil_max_camber_pos")));
    stabiliser_airfoil_max_thickness = ...
        der_params(find(strcmp(der_param_names, "horizontal_stabiliser_airfoil_max_thickness")));


    stabiliser_thickness = parameters(find(strcmp(param_names, "horizontal_stabiliser_thickness")));

    plastic_thickness = parameters(find(strcmp(param_names, "plastic_thickness")));

    mesh_resolution = parameters(find(strcmp(param_names, "mesh_resolution")));

    num_points = 11;

    [airfoil_upper, airfoil_lower] = generate_NACA_airfoil(stabiliser_airfoil_max_camber, ...
        stabiliser_airfoil_max_camber_pos, stabiliser_airfoil_max_thickness, ...
        stabiliser_thickness, num_points);

    % plot(airfoil_upper(:, 1), airfoil_upper(:, 2), 'r-', ...
    % airfoil_upper(:, 1), airfoil_upper(:, 2), 'r-');
    % hold on;
    % 
    % plot(airfoil_lower(:, 1), airfoil_lower(:, 2), 'r-', ...
    %    airfoil_lower(:, 1), airfoil_lower(:, 2), 'r-');
    % hold off;
   
    %Now generate a fuselage base profile:
    airfoil_profile = profile(airfoil_upper, airfoil_lower, 1, plastic_thickness, mesh_resolution);


end