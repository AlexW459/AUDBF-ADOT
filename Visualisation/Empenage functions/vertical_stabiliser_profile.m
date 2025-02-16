%Name: Alex
%Date: 22/8/24
%Description: Creates the profile that can be extruded to produce the
%vertical stabiliser on the empennage

function airfoil_profile = vertical_stabiliser_profile(parameters, param_names, der_params, der_param_names)
    stabiliser_airfoil_max_camber = ...
        der_params(find(strcmp(der_param_names, "vertical_stabiliser_airfoil_max_camber")));
    stabiliser_airfoil_max_camber_pos = ...
        der_params(find(strcmp(der_param_names, "vertical_stabiliser_airfoil_max_camber_pos")));
    stabiliser_airfoil_max_thickness = ...
        der_params(find(strcmp(der_param_names, "vertical_stabiliser_airfoil_max_thickness")));

    stabiliser_thickness = parameters(find(strcmp(param_names, "vertical_stabiliser_thickness")));

    plastic_thickness = parameters(find(strcmp(param_names, "plastic_thickness")));

    mesh_resolution = parameters(find(strcmp(param_names, "mesh_resolution")));

    num_points = 11;

    [airfoil_upper, airfoil_lower] = generate_NACA_airfoil(stabiliser_airfoil_max_camber, ...
        stabiliser_airfoil_max_camber_pos, stabiliser_airfoil_max_thickness, ...
        stabiliser_thickness, num_points);


    airfoil_profile = profile(airfoil_upper, airfoil_lower, 1, plastic_thickness, mesh_resolution);

end