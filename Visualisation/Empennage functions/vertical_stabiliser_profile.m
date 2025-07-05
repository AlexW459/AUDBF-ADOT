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
    stabiliser_root_chord = der_params(find(strcmp(der_param_names, "vertical_stabiliser_chord")));


    stabiliser_thickness = parameters(find(strcmp(param_names, "vertical_stabiliser_thickness")));

    plastic_thickness = parameters(find(strcmp(param_names, "plastic_thickness")));

    mesh_resolution = parameters(find(strcmp(param_names, "mesh_resolution")));

    num_points = ceil(stabiliser_root_chord*mesh_resolution);

    airfoil_points = generate_NACA_airfoil(stabiliser_airfoil_max_camber, ...
        stabiliser_airfoil_max_camber_pos, stabiliser_airfoil_max_thickness, ...
        stabiliser_thickness, num_points);


    airfoil_profile = profile(airfoil_points, 1, plastic_thickness);

end