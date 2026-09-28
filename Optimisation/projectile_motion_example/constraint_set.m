%Name:    Isaac Nakone, Muhammad Rabay, Alec Vitalievich
%Date:    06/09/2024
%Purpose: Demonstrate how to do optimisation with constraints.

classdef constraint_set < handle

    properties


        constraint_names = [];     %This is a vector of constraint name strings

        local_variables = [];     %These are the names of the local variables involved
                                  % in the constraint.
        is_equality_constraint = [];

        global_variables = [];  %This is all the variables mentioned in the constraint function, no repeats.

    end

    methods 

        function determine_constraint_names(obj)
            constraint_files = dir( 'constraint_functions' );
            obj.constraint_names = { constraint_files.name };
            
            
            constraint_names_new = {};
            
            for name_index = 1:size(obj.constraint_names,2)
            
                str_length = strlength(obj.constraint_names{name_index});
            
                if (str_length > 2)
            
                    pos = str_length;
            
                    file_extension = extract(obj.constraint_names{name_index}, pos );
            
            
            
                    if (strcmp(strcat(".",file_extension), ".m"))
                        constraint_names_new = [constraint_names_new; obj.constraint_names{name_index}];
                    end
            
                end
                
            end
            
            obj.constraint_names = constraint_names_new;
            
            
            for name_index = 1:size(obj.constraint_names,1)
            
                str_length = strlength(obj.constraint_names{name_index} );
            
                obj.constraint_names{name_index}([str_length-1, str_length]) = []; %Remove the extension. 
            
            end


        end



        function determine_input_names_and_constraint_types(obj)

            %Count the number of constraints:

            num_constraints = size(obj.constraint_names,1);

            %Preallocate local_variables as a cell:
            obj.local_variables = cell(num_constraints,1);


            cd constraint_functions\;



            for constraint_index = 1:num_constraints


               [~,obj.local_variables{constraint_index}, obj.is_equality_constraint(constraint_index)] = ...
                            eval(strcat(obj.constraint_names{constraint_index}, "();"));
               %This will get the local variable names returned by the
               %function as well as the type of constraint.

               

            end

            cd ..\;
            
        end

        function determine_global_variables(obj)

            for constraint_index = 1:size(obj.constraint_names,1)
                local_variables_constraint_index = obj.local_variables{constraint_index};

                num_local_variables = size(local_variables_constraint_index,1);

                for local_variable_index = 1:num_local_variables
                    obj.global_variables = [obj.global_variables;...
                        local_variables_constraint_index{local_variable_index }];

                end


            end

            obj.global_variables = unique(obj.global_variables,'rows','sorted');

            
        end


        function [inequality_constraints, equality_constraints] = problem_constraints(obj, x,constants)

            inequality_constraints = [];
            equality_constraints = [];

            

            cd constraint_functions\;


            for constraint_index = 1:size(obj.constraint_names,1)
                lvci = obj.local_variables{constraint_index}; 
                input_arguments_str = "( x , constants );";

  
                

                

                local_function_call = strcat(obj.constraint_names{constraint_index}, input_arguments_str);

                if (obj.is_equality_constraint(constraint_index) == true)

                    equality_constraints(constraint_index,1) = eval(local_function_call);

                else
                    inequality_constraints(constraint_index,1) = eval(local_function_call);

                end


            end

            cd ..\;



        end

    end

end