%Name: Alex
%Date: 9/8/24
%Description: Creates the profile that will be extruded to produce the
%motor pod

function output_profile = motor_pod_profile(parameters, param_names, der_params, der_param_names)

    motor_width = der_params(find(strcmp(der_param_names, "motor_width")));

    plastic_thickness = ...
        parameters(find(strcmp(param_names, "plastic_thickness")));

    pod_width = 2*plastic_thickness+motor_width;
    
    mesh_resolution = ...
        parameters(find(strcmp(param_names, "mesh_resolution")));



    num_points = 6;

    t_star = linspace(0, 180, num_points)';


    B_upper = [-pod_width*0.5*cosd(t_star), pod_width*0.5*sind(t_star)+0.25*pod_width];
    B_lower = [pod_width*0.5*cosd(t_star), -pod_width*0.5*sind(t_star)-pod_width*0.25];



    output_profile = profile(B_upper, B_lower, -1, plastic_thickness, mesh_resolution*4);

end