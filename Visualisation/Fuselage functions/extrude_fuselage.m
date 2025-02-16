%Name: Alex
%Date: 3/8/24
%Description: Extrudes the fuselage profile to produce an extrusion object
%based on aircraft parameters


function [fuselage_extrusion, pivot_point] = extrude_fuselage(profiles, ...
    profile_names, parameters, param_names, der_params, der_param_names, density)

    fuselage_height = ...
        parameters(find(strcmp(param_names, "fuselage_height")));
    fuselage_length = ...
        parameters(find(strcmp(param_names, "fuselage_length")));
    nosecone_length = ...
        parameters(find(strcmp(param_names, "nosecone_length")));
    nosecone_tip_scale = ...
        parameters(find(strcmp(param_names, "nosecone_tip_scale")));
    nosecone_offset = ...
        parameters(find(strcmp(param_names, "nosecone_offset")));

    mesh_resolution = ...
        parameters(find(strcmp(param_names, "mesh_resolution")));

    fuselage_base_profile = profiles{find(strcmp(profile_names, "fuselage_profile"))};

    nosecone_solid_height = nosecone_tip_scale*fuselage_height;

    %Finds the length of the hollow section
    Q2xPos = (nosecone_solid_height*nosecone_length)/(fuselage_height-0.5*nosecone_solid_height);
    nosecone_hollow_length = nosecone_length - 0.5*Q2xPos;

    %Find slope of tangent to the point 
    upperSlope = ((nosecone_solid_height-fuselage_height)*0.5+nosecone_offset)...
    /(nosecone_hollow_length);

    Q2 = [Q2xPos+nosecone_hollow_length, Q2xPos*upperSlope+nosecone_solid_height*0.5+nosecone_offset*0.5];

    %Defines the start and end of the beizer curve as the points at the end of
    %the hollow section
    Q1 = [nosecone_hollow_length,-nosecone_solid_height*0.5+nosecone_offset];
    Q3 = [nosecone_hollow_length,nosecone_solid_height*0.5+nosecone_offset];

    num_t = ceil(mesh_resolution*Q2xPos*0.5);

    t_star = linspace(0,0.5,num_t);

    [bez_profile, x_sample] = bezier_curve_profile(t_star, Q1,Q2,Q3);


    battery_pos = ...
        parameters(find(strcmp(param_names, "battery_pos")));

    battery_mass = ...
        der_params(find(strcmp(der_param_names, "battery_mass")));

    x_sample = [0, fuselage_length];

    %Finds the side profile of the extrusion
    side_profile = [fuselage_height*0.5, fuselage_height*(-0.5)];

    side_profile_matrix = repmat(side_profile, [2, 1]);

   
    fuselage_extrusion = extrusion(fuselage_base_profile, x_sample, ...
        side_profile_matrix, 0, density, battery_mass, battery_pos*[0, 0, fuselage_length]);

    pivot_point = [0, 0, 0];

end