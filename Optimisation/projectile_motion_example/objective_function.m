%Name:    Isaac Nakone, Muhammad Rabay, Alec Vitalievich
%Date:    06/09/2024
%Purpose: The objective function actually must set the projectile object
%         based on the inputs:



function objective_value = objective_function(my_projectile, x)


    %Assume x comes in as a large array:

    my_projectile.set_variable_values_cell(x); %This designs my_projectile 
    %                                           so that it will have a cell with the values in x.

    my_projectile.generate_struct();  %This generates variables struct.

    objective_value = -my_projectile.variables_struct.range; % Let's say we want to maximise the range.
    

    %NOTE: Don't forget to do the previous steps.

end