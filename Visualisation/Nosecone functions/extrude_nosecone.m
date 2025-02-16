%Name: Alex
%Date: 2/8/24
%Description: Function that extrudes the nosecone based on the aircraft parameters

%Function that generates the hollow section of the nosecone
function [nosecone_extrusion, pivot_point] = extrude_nosecone(profiles, ...
    profile_names, parameters, param_names, der_params, der_param_names, density)


    fuselage_height = ...
        parameters(find(strcmp(param_names, "fuselage_height")));
    nosecone_length = ...
        parameters(find(strcmp(param_names, "nosecone_length")));
    nosecone_tip_scale = ...
        parameters(find(strcmp(param_names, "nosecone_tip_scale")));
    nosecone_offset = ...
        parameters(find(strcmp(param_names, "nosecone_offset")));

    mesh_resolution = ...
        parameters(find(strcmp(param_names, "mesh_resolution")));
    
    nosecone_solid_height = nosecone_tip_scale*fuselage_height;

    %Finds the length of the hollow section
    Q2xPos = (nosecone_solid_height*nosecone_length)/(fuselage_height-0.5*nosecone_solid_height);
    nosecone_hollow_length = nosecone_length - 0.5*Q2xPos;


    %Find slope of tangent to the point 
    upperSlope = ((nosecone_solid_height-fuselage_height)*0.5+nosecone_offset)...
    /(nosecone_hollow_length);

    Q2 = [Q2xPos+nosecone_hollow_length, Q2xPos*upperSlope+nosecone_solid_height*0.5+nosecone_offset*0.5];

    %Defines the start and end of the belzer curve as the points at the end of
    %the hollow section
    Q1 = [nosecone_hollow_length,-nosecone_solid_height*0.5+nosecone_offset];
    Q3 = [nosecone_hollow_length,nosecone_solid_height*0.5+nosecone_offset];

    num_t = ceil(mesh_resolution*Q2xPos*0.5);

    t_star = linspace(0,0.5,num_t);

    [bez_profile, x_sample] = bezier_curve_profile(t_star,Q1,Q2,Q3);

    straight_extrusion_upper = linspace(fuselage_height*0.5, ...
        nosecone_solid_height*0.5+nosecone_offset, ceil(nosecone_hollow_length*mesh_resolution))';
    straight_extrusion_lower = linspace(-fuselage_height*0.5, ...
        -nosecone_solid_height*0.5+nosecone_offset, ceil(nosecone_hollow_length*mesh_resolution))';

    %Finds the side profile of the extrusion
    side_profile = [straight_extrusion_upper, ...
        straight_extrusion_lower];
    
    side_profile = [side_profile(1:end-1, :); bez_profile];

    extrude_marks = linspace(0, nosecone_hollow_length, ceil(nosecone_hollow_length*mesh_resolution));

    extrude_marks = [extrude_marks(1:end-1), x_sample];

    %Creates the extrusion
    fuselage_profile = profiles{find(strcmp(profile_names, "fuselage_profile"))};

    nosecone_extrusion = extrusion(fuselage_profile, extrude_marks(1:end-1), ...
    side_profile(1:end-1, :), 0, density);

    
    pivot_point = [0, 0, 0];

end