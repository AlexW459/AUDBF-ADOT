%Name: Alex
%Date: 3/8/24
%Description: Generates the profile of the wing

function airfoil_profile = wing_profile(parameters, param_names, der_params, der_param_names)
    wing_airfoil_max_camber = der_params(find(strcmp(der_param_names, "wing_airfoil_max_camber")));
    wing_airfoil_max_camber_pos = der_params(find(strcmp(der_param_names, "wing_airfoil_max_camber_pos")));
    wing_airfoil_max_thickness = der_params(find(strcmp(der_param_names, "wing_airfoil_max_thickness")));

    wing_root_thickness = parameters(find(strcmp(param_names, "wing_root_thickness")));

    plastic_thickness = parameters(find(strcmp(param_names, "plastic_thickness")));


    mesh_resolution = parameters(find(strcmp(param_names, "mesh_resolution")));

    %num_points must be an even number
    num_points = 16;

    [airfoil_upper, airfoil_lower] = generate_NACA_airfoil(wing_airfoil_max_camber, ...
        wing_airfoil_max_camber_pos, wing_airfoil_max_thickness, ...
        wing_root_thickness, num_points);

    % plot(airfoil_upper(:, 1), airfoil_upper(:, 2),'ko');
    % hold on;
    % plot(airfoil_lower(:, 1), airfoil_lower(:, 2),'ko');
    % hold off;
    % 

    %Now generate an airfoil base profile:
    airfoil_profile = profile(airfoil_upper, airfoil_lower,...
        1, plastic_thickness, mesh_resolution);

end