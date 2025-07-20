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
    
    nosecone_tip_height = nosecone_tip_scale*fuselage_height;

    tail_length = ...
         parameters(find(strcmp(param_names, "tailcone_length")));
    tailcone_tip_scale = ...
         parameters(find(strcmp(param_names, "tailcone_tip_scale")));
    tailcone_tip_height = tailcone_tip_scale*fuselage_height;

    %Finds the length of the hollow section
    Q2xPos = (nosecone_tip_height*nosecone_length)/(fuselage_height-0.5*nosecone_tip_height);
    nosecone_hollow_length = nosecone_length - 0.5*Q2xPos;

    %Find slope of tangent to the point 
    upperSlope = ((nosecone_tip_height-fuselage_height)*0.5+nosecone_offset)...
    /(nosecone_hollow_length);

    Q2 = [Q2xPos+nosecone_hollow_length, Q2xPos*upperSlope+nosecone_tip_height*0.5+nosecone_offset*0.5];

    %Defines the start and end of the beizer curve as the points at the end of
    %the hollow section
    Q1 = [nosecone_hollow_length,-nosecone_tip_height*0.5+nosecone_offset];
    Q3 = [nosecone_hollow_length,nosecone_tip_height*0.5+nosecone_offset];

    num_t = ceil(mesh_resolution*Q2xPos);

    t_star = linspace(0,0.5,num_t);

    [nose_bez_profile, nose_x_sample] = bezier_curve_profile(t_star, Q1,Q2,Q3);

    %Finds the length of the hollow section
    Q2xPos = (tailcone_tip_height*tail_length)/(fuselage_height-0.5*tailcone_tip_height);

    Q2 = [tail_length+0.5*Q2xPos, fuselage_height*0.5];
    tail_straight_length = tail_length - 0.5*Q2xPos;

    %Defines the start and end of the belzer curve as the points at the end of
    %the hollow section
    Q1 = [tail_straight_length, fuselage_height*0.5-tailcone_tip_height];
    Q3 = [tail_straight_length, fuselage_height*0.5];

    num_t = ceil(Q2xPos*2*mesh_resolution);

    t_star = linspace(0,0.5,num_t);

    [tail_bez_profile, tail_x_sample] = bezier_curve_profile(t_star,Q1,Q2,Q3);
    
    %plot(tail_x_sample, tail_bez_profile(:, 1), tail_x_sample, tail_bez_profile(:, 2));

    battery_pos = ...
        parameters(find(strcmp(param_names, "battery_pos")));

    battery_mass = ...
        der_params(find(strcmp(der_param_names, "battery_mass")));

    x_sample = [-1*tail_x_sample(end-1:-1:1), 0, fuselage_length, nose_x_sample(1:end-1)+fuselage_length];

    %Finds the side profile of the extrusion
    side_profile_matrix = [tail_bez_profile(end-1:-1:1, :);
        fuselage_height*0.5, -fuselage_height*0.5;
        fuselage_height*0.5, -fuselage_height*0.5;...
        nose_bez_profile(1:end-1, :)];

    %plot(x_sample', side_profile_matrix(:, 1), x_sample', side_profile_matrix(:, 2));
   
    fuselage_extrusion = extrusion(fuselage_base_profile, x_sample, ...
        side_profile_matrix, 0, density, battery_mass, battery_pos*[0, 0, fuselage_length]);

    

    pivot_point = [0, 0, 0];

end