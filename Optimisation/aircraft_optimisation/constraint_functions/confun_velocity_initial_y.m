%Name:    Isaac Nakone, Muhammad Rabay, Alec Vitalievich
%Date:    06/09/2024
%Purpose: Demonstrate the form of a constraint function.


function [nonlcon_entry, input_names, is_equality_constraint ] = confun_velocity_initial_y(input_values, constants)

    input_names = {"velocity_initial";
                   "velocity_initial_y";
                   "angle_launch"};  %Note the same order as function inputs.

    is_equality_constraint = true;

    if (nargin == 0)

        nonlcon_entry = [];


    else

        left_hand_side  = input_values.velocity_initial_y;                  %The left side of the constraint function.
        right_hand_side = input_values.velocity_initial*sind(input_values.angle_launch); %The right side of the constraint function.
        nonlcon_entry   = left_hand_side - right_hand_side;    %The difference ==> so that we get zero for 
                                                               % an equality constraint.
    end
end