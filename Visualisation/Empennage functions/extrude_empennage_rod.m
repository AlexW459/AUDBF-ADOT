%Name: Alex
%Date: 19/8/24
%Description: Extrudes the supports that connect to the empennage

function [empennage_support, pivot_point] = extrude_empennage_rod(profiles, ...
    profile_names, parameters, param_names, der_params, der_param_names, density)
    circular_profile = profiles{find(strcmp(profile_names, "empennage_support_profile"))};

    rod_width = ...
        der_params(find(strcmp(der_param_names, "empennage_rod_width")));
    rod_length = der_params(find(strcmp(der_param_names, "empennage_rod_length")));


    extrude_marks = [0, rod_length];

    support_side_profile = [rod_width*0.5, -0.5*rod_width; ...
                            rod_width*0.5, -0.5*rod_width];

  
    empennage_support = extrusion(circular_profile, extrude_marks, support_side_profile, 0, density);

    pivot_point = [0, 0, 0];


end