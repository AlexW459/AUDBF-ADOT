%Name:    Isaac Nakone, Muhammad Rabay, Alec Vitalievich
%Date:    06/09/2024
%Purpose: Demonstrate how to do optimisation with constraints.

classdef projectile < handle
    properties

        constraint_list = [];

        variables = {};

        variables_struct = struct;

        lower_bounds = [];

        lower_bounds_struct = struct;

        upper_bounds = [];

        upper_bounds_struct = struct;

        constants = struct;

        initial_value = [];


    end




    methods 

        function obj = projectile(constants)

            obj.constants = constants;

            obj.constraint_list = constraint_set;
        
            obj.constraint_list.determine_constraint_names();
            
            obj.constraint_list.determine_input_names_and_constraint_types();
            
            obj.constraint_list.determine_global_variables();

            for variable_index = 1:size(obj.constraint_list.global_variables,1)

                obj.variables{variable_index,1}.name  = obj.constraint_list.global_variables{variable_index};
                obj.variables{variable_index,1}.value = 0;

            end


            obj.set_variable_values_cell(nan(size(obj.variables,1),1));

            obj.generate_struct();

            obj.lower_bounds_struct = obj.variables_struct;

            obj.upper_bounds_struct = obj.variables_struct;
            
            

        end

        function set_variable_values_cell(obj,input_values)

            for variable_index = 1:size(obj.constraint_list.global_variables,1)  
                obj.variables{variable_index,1}.value = input_values(variable_index);

            end


        end

        function generate_struct(obj)

            %produce a cell of names:

            names = {};
            values = {};

            for i = 1:size(obj.variables,1) 
                names{i}  = obj.variables{i}.name;
                values{i} = obj.variables{i}.value;
            end

            
            obj.variables_struct = cell2struct(values,names,2);



        end


        function [inequality_constraints, equality_constraints] = problem_constraints(obj,x)

            obj.set_variable_values_cell(x); %This designs my_projectile 
            %                                           so that it will have a cell with the values in x.
        
            obj.generate_struct();  %This generates variables struct.


            [inequality_constraints, equality_constraints] = ...
                obj.constraint_list.problem_constraints(obj.variables_struct, obj.constants);

        end


        function generate_initial_values(obj)

            for variable_index = 1:size(obj.variables,1)

                name = obj.variables{variable_index}.name;
                obj.lower_bounds(variable_index) = eval(strcat("obj.lower_bounds_struct.",name));
                obj.upper_bounds(variable_index) = eval(strcat("obj.upper_bounds_struct.",name));
                obj.initial_value(variable_index) = 0.5*(obj.lower_bounds(variable_index)+obj.upper_bounds(variable_index));

            end

        end


        function optimise(obj)

            x = fmincon(@(x)objective_function(obj,x),...
             obj.initial_value,[],[],[],[], ...
             obj.lower_bounds,obj.upper_bounds, ...
             @(x)obj.problem_constraints(x)); 



        end


        function display_variable_summary(obj)

            disp("A summary of the optimised design:");

            for variable_index = 1:size(obj.variables,1)

                name = obj.variables{variable_index}.name;

                disp(strcat("(", num2str(variable_index), "): ", name, " is ", num2str(obj.variables{variable_index}.value)));


            end

            


        end


    end



end