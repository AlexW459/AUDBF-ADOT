%Name:    Isaac Nakone, Muhammad Rabay, Alec Vitalievich
%Date:    06/09/2024
%Purpose: Demonstrate the form of a constraint function.


function [nonlcon_entry, input_names, is_equality_constraint ] = confun_range(input_values,constants)

    input_names = {"range"; 
                   "velocity_initial_x";
                   "velocity_initial_y";
                   "position_initial_y"};

    is_equality_constraint = true;


    if (nargin == 0)
        nonlcon_entry = [];
    else
    
        left_hand_side  = input_values.range;                                              %The left side of the constraint function.
        right_hand_side = (input_values.velocity_initial_x/constants.acceleration_gravity)*...
                          (input_values.velocity_initial_y+sqrt(input_values.velocity_initial_y^2 + ...
                           2*constants.acceleration_gravity*input_values.position_initial_y));       %The right side of the constraint function.
        nonlcon_entry   = left_hand_side - right_hand_side;                   %The difference 


    end



end