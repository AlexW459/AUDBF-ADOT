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
    stabiliser_root_chord = der_params(find(strcmp(der_param_names, "horizontal_stabiliser_chord")));


    stabiliser_thickness = parameters(find(strcmp(param_names, "horizontal_stabiliser_thickness")));

    plastic_thickness = parameters(find(strcmp(param_names, "plastic_thickness")));

    mesh_resolution = parameters(find(strcmp(param_names, "mesh_resolution")));

    num_points = ceil(mesh_resolution*stabiliser_root_chord);

    airfoil_points = generate_NACA_airfoil(stabiliser_airfoil_max_camber, ...
        stabiliser_airfoil_max_camber_pos, stabiliser_airfoil_max_thickness, ...
        stabiliser_thickness, num_points);

    % plot(airfoil_points(:, 1), airfoil_points(:, 2), 'r-');
    % hold on;

   
    %Now generate a fuselage base profile:
    airfoil_profile = profile(airfoil_points, 1, plastic_thickness);

end