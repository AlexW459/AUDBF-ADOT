%Name: Alex
%Date: 19/8/24
%Description: Creates the profile that can be extruded to produce the
%empenage rods

function output_profile = empennage_boom_profile(parameters, param_names, der_params, der_param_names)
    boom_thickness = 0.04;
    plastic_thickness = parameters(find(strcmp(param_names, "plastic_thickness")));

    mesh_resolution = parameters(find(strcmp(param_names, "mesh_resolution")));

    num_points = ceil(mesh_resolution*0.5*boom_thickness*pi*4);

    max_degrees = 180;

    %Generates a circle of points
    t = linspace(0, max_degrees, num_points)';

    
    B_upper = [-0.5*boom_thickness*cosd(t), 0.5*boom_thickness*sind(t)];
    B_lower = [0.5*boom_thickness*cosd(t), -0.5*boom_thickness*sind(t)];

    B = [B_upper(1:end-1, :); B_lower(1:end-1, :)];

    %plot(B(:, 1), B(:, 2));

    output_profile = profile(B, plastic_thickness);

end
