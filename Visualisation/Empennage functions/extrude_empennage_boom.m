%Name: Alex
%Date: 19/8/24
%Description: Extrudes the supports that connect to the empennage

function [empennage_boom, pivot_point] = extrude_empennage_boom(profiles, ...
    profile_names, parameters, param_names, der_params, der_param_names, density)
    circular_profile = profiles{find(strcmp(profile_names, "empennage_boom_profile"))};

    boom_width = 0.04;
    boom_length = der_params(find(strcmp(der_param_names, "empennage_boom_length")));

    rod_width = ...
        der_params(find(strcmp(der_param_names, "empennage_rod_width")));
    rod_length = der_params(find(strcmp(der_param_names, "empennage_rod_length")));


    extrude_marks = [0, -boom_length+0.01, -boom_length, -boom_length-rod_length];

    boom_side_profile = [boom_width*0.5, -0.5*boom_width; ...
                            boom_width*0.5, -0.5*boom_width;
                            rod_width*0.5, -0.5*rod_width;
                            rod_width*0.5, -0.5*rod_width];

    empennage_boom = extrusion(circular_profile, extrude_marks, boom_side_profile, 0, density);

    pivot_point = [0, 0, 0];


end