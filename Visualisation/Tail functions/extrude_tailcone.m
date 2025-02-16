%Name: Alex
%Date: 8/8/24
%Description: Function that extrudes the tailcone

%Function that generates the tailcone
function [tailcone_extrusion, pivot_point] = extrude_tailcone(profiles, profile_names, ...
    parameters, param_names, der_params, der_param_names, density)
    
    fuselage_height = ...
         parameters(find(strcmp(param_names, "fuselage_height")));
    tail_length = ...
         parameters(find(strcmp(param_names, "tailcone_length")));
    tailcone_tip_scale = ...
         parameters(find(strcmp(param_names, "tailcone_tip_scale")));
     
    tailcone_tip_height = tailcone_tip_scale*fuselage_height;
    mesh_resolution = ...
        parameters(find(strcmp(param_names, "mesh_resolution")));
    
    %Finds the length of the hollow section
    Q2xPos = (tailcone_tip_height*tail_length)/(fuselage_height-0.5*tailcone_tip_height);

    Q2 = [tail_length+0.5*Q2xPos, fuselage_height*0.5];
    tail_straight_length = tail_length - 0.5*Q2xPos;

    %Defines the start and end of the belzer curve as the points at the end of
    %the hollow section
    Q1 = [tail_straight_length, fuselage_height*0.5-tailcone_tip_height];
    Q3 = [tail_straight_length, fuselage_height*0.5];

    %Creates array of values of t
    num_t = ceil(Q2xPos*mesh_resolution);

    t_star = linspace(0,0.5,num_t);

    [bez_profile, x_sample] = bezier_curve_profile(t_star,Q1,Q2,Q3);
    

    extrude_marks = linspace(0, tail_straight_length, ceil(mesh_resolution*tail_straight_length));
    
    extrude_marks = [extrude_marks(1:end-1), x_sample];

    straight_extrusion_upper = linspace(fuselage_height*0.5, ...
        fuselage_height*0.5, ceil(tail_straight_length*mesh_resolution))';
    straight_extrusion_lower = linspace(-fuselage_height*0.5, ...
         fuselage_height*0.5-tailcone_tip_height, ceil(tail_straight_length*mesh_resolution))';

    %Finds the side profile of the extrusion
    side_profile = [straight_extrusion_upper, straight_extrusion_lower];
    
    side_profile = [side_profile(1:end-1, :); bez_profile];


    %Gets the profile
    fuselage_profile = profiles{find(strcmp(profile_names, "fuselage_profile"))};

    fuselage_profile.translate_direction = -1;

    tailcone_extrusion = extrusion(fuselage_profile, extrude_marks(1:(end-1)), ...
    side_profile(1:(end-1), :), 0, density);


    %plot(extrude_marks(1:end-1), side_profile(1:end-1, 1), extrude_marks(1:end-1), side_profile(1:end-1, 2));

    pivot_point = [0, 0, 0];

    %tailcone_extrusion.show_facets();
end