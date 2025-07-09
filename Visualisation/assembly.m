%Name: Alex, Isaac
%Date: 15/07/2024
%Description: Class describing an assembly of shapes

classdef assembly < handle
    properties
    
        
        %Contains extrusion objects
        parts = {};
        num_parts = 0;

        part_COMs = [];
        part_masses = [];

        %Stores list of possible materials
        material_table = [];

        %Stores the materials the compose each part
        material_indices = [];

        %Stores list of possible motors, all values in SI units
        motor_table = [];

        %Stores list of possible batteries
        battery_table = [];

        %Stores list of possible airfoils
        airfoil_table = [];


        %Contains extrusion objects orientated into the correct posiitons
        transformed_parts = {};
        
        %Contains the tree object that links the parts together
        part_tree;


        %Contains a list of parameters that are used to define the
        %constraints e.g. fuselage length could be one parameter, which 
        %is then used to define the constraint that the nosecone must sit 
        %at the end of the fuselage i.e. half the length of the fuselage
        %away from (0,0,0) assuming the centre of the fuselage is the origin
        %Matrix is y rows, 1 column
        aircraft_parameters = [];

        parameter_names = [];

        der_param_function = [];

        derived_parameters = [];

        derived_param_names = [];


        %Contains a list of functions (one per part) which define the
        %location of the part in terms of all of the aircraft parameters
        %Cell vector is 1 rows, x columns
        constraints = {};

        %Contains a list of functions that generate the profile of each
        %part based on the aircraft parameters
        profile_constructors = {};

        %Contains a list of functions that generate the extrusion of each
        %part based on the aircraft parameters
        extrusion_constructors = {};
        
        %Stores points around which parts are rotated. These coordinates
        %are in the part's coordinate system
        pivot_points = [];


        %Stores all of the profiles that are extruded to produce the parts
        part_profiles = {};

        %Stores the names of each profile so they can be found easily
        profile_names = [];

        %Contains the locations of each part, calculated using the
        %parameters and the constraint functions for each part. The
        %location of a part is relative to its parent part
        %Matrix is x rows, 3 columns
        part_locations = [];

        %Stores the rotations to be performed on each part, calculated using 
        %the parameters and the constraint function for each part. Parts are
        %rotated before being translated to their correct locations
        %Matrix is x rows, 4 columns. The first 3 columns are a vector
        %describing the axis of rotation, and the last column is the number
        %of degrees of rotation
        part_rotations = [];

        %Total mass of aircraft
        mass = 0;

        %COM of aircraft
        COM = zeros(1, 3);

    end

    methods
        function obj = assembly(base_extrude_function, base_constraint, base_part_name,...
            base_part_material_index, profile_functions, profile_name_list, parameters, ...
            param_names, derived_param_function, material_table, motor_table, ...
            battery_table, airfoil_table)
            
            %Creates tree structure to store parts
            obj.part_tree = tree(base_part_name);

            obj.material_indices = base_part_material_index;

            obj.parts = cell(1, 1);
            obj.num_parts = 1;

            obj.aircraft_parameters = parameters;
            obj.parameter_names = param_names;
            obj.constraints = {base_constraint};

            obj.material_table = material_table;
            
            obj.airfoil_table = airfoil_table;

            obj.motor_table = motor_table;


            %Converts units to SI units
            for i = 1:size(obj.motor_table.Properties.VariableNames, 2)
                if (strcmp(convertCharsToStrings(obj.motor_table.Properties.VariableUnits{i}), "mm"))
                    obj.motor_table{:, i} = obj.motor_table{:, i}/1000;
                    obj.motor_table.Properties.VariableUnits{i} = 'm';
                end
            
                if (strcmp(obj.motor_table.Properties.VariableUnits{i}, "g") && ...
                        strcmp(convertCharsToStrings(obj.motor_table.Properties.VariableNames{i}), "Weight"))
                    obj.motor_table{:, i} = obj.motor_table{:, i}/1000;
                    obj.motor_table.Properties.VariableUnits{i} = 'kg';
                    
                end
            
                if (strcmp(convertCharsToStrings(obj.motor_table.Properties.VariableUnits{i}), "g") && ...
                        strcmp(obj.motor_table.Properties.VariableNames{i}, "Thrust"))
                    obj.motor_table{:, i} = obj.motor_table{:, i}*9.8/1000;
                    obj.motor_table.Properties.VariableUnits{i} = 'N';
                    
                end
            
            end

            obj.battery_table = battery_table;

            %Converts units to SI units
            for i = 1:size(obj.battery_table.Properties.VariableNames, 2)

                if (strcmp(obj.battery_table.Properties.VariableUnits{i}, "g") && ...
                        strcmp(convertCharsToStrings(obj.battery_table.Properties.VariableNames{i}), "Weight"))
                    obj.battery_table{:, i} = obj.battery_table{:, i}/1000;
                    obj.battery_table.Properties.VariableUnits{i} = 'kg';
                    
                end
            
                if (strcmp(convertCharsToStrings(obj.battery_table.Properties.VariableUnits{i}), "mAh"))
                    obj.battery_table{:, i} = obj.battery_table{:, i}/1000;
                    obj.battery_table.Properties.VariableUnits{i} = 'Ah';
                end
            
            end

            obj.der_param_function = derived_param_function;

            [obj.derived_parameters, obj.derived_param_names] = ...
                obj.der_param_function(obj.aircraft_parameters, ...
                obj.parameter_names, obj.motor_table, obj.battery_table, ...
                obj.airfoil_table);

            %Assign variables based on given parameters
            obj.profile_constructors = profile_functions;
            obj.profile_names = profile_name_list;
            obj.extrusion_constructors = {base_extrude_function};

            %Creates part extrusion
            obj.part_profiles = cell(1, size(profile_functions, 2));

            base_transformations = base_constraint(obj.aircraft_parameters, obj.parameter_names);

            obj.part_locations = base_transformations(1:3);
            obj.part_rotations = base_transformations(4:7);


        end

        function addPart(obj, new_extrusion_function, part_name, parent_name, ...
                new_constraint, material_index)

            obj.num_parts = obj.num_parts + 1;
            %Adds the properties of the new part to the aircraft parameters
            obj.extrusion_constructors = [obj.extrusion_constructors, {new_extrusion_function}];
            obj.constraints = [obj.constraints, {new_constraint}];


            %Adds material to list
            obj.material_indices = [obj.material_indices, material_index];


            %Adds part to tree structure
            obj.part_tree.add_child_to_parent(part_name, parent_name);
        end

        function compute_params(obj)
            [obj.derived_parameters, obj.derived_param_names] = ...
                obj.der_param_function(obj.aircraft_parameters, ...
                obj.parameter_names, obj.motor_table, obj.battery_table, ...
                obj.airfoil_table);

        end

        function construct_profiles(obj)
            %Constructs profiles
            profiles = obj.part_profiles;
            for i = 1:size(obj.profile_constructors, 2)
                current_constructor = obj.profile_constructors{i};
                profiles{i} = current_constructor(obj.aircraft_parameters, ...
                    obj.parameter_names, obj.derived_parameters, obj.derived_param_names);
            end

            obj.part_profiles = profiles;

        end

        function construct_parts(obj)
            obj.parts = cell(obj.num_parts, 1);
            obj.pivot_points = zeros(obj.num_parts, 3);
            obj.part_masses = zeros(obj.num_parts, 1);
            obj.part_COMs = zeros(obj.num_parts, 3);

            temp_part_masses = obj.part_masses;
            temp_parts = obj.parts;
            temp_pivot_points = obj.pivot_points;
            temp_part_COMs = obj.part_COMs;
            

            %Constructs parts by extruding profiels
            for i = 1:obj.num_parts
                 current_constructor = obj.extrusion_constructors{i};

                 current_density = obj.material_table{obj.material_indices(i), 2};

                 [new_part, new_pivot_point] = current_constructor(obj.part_profiles,...
                 obj.profile_names, obj.aircraft_parameters, obj.parameter_names, ...
                 obj.derived_parameters, obj.derived_param_names, current_density);
                 temp_parts{i} = new_part;

                 
                 temp_pivot_points(i, :) = new_pivot_point;

                 [new_part_COM, new_part_mass] = new_part.find_COM();

                 temp_part_COMs(i, :) = new_part_COM;
                 temp_part_masses(i) = new_part_mass;


                 temp_part_masses(i) = new_part_mass;
            end

            obj.part_masses = temp_part_masses;
            obj.parts = temp_parts;
            obj.pivot_points = temp_pivot_points;
            obj.part_COMs = temp_part_COMs;

            obj.mass = sum(obj.part_masses);

        end

        function compute_constraints(obj)
            obj.part_locations = zeros(obj.num_parts, 3);
            obj.part_rotations = zeros(obj.num_parts, 4);
            
            for i = 1:obj.num_parts

                %Produces vector of 7 elements defining necessary
                %translations and rotations
                current_constraint = obj.constraints{i};
                transformations = current_constraint(obj.aircraft_parameters, obj.parameter_names, ...
                    obj.derived_parameters, obj.derived_param_names);


                obj.part_locations(i, :) = transformations(1:3);
                
                obj.part_rotations(i, :) = transformations(4:7);
            end

        end


        function transform_parts(obj)
            %Create a new vector of parts that can be transformed to their
            %correct position
            obj.transformed_parts = obj.parts;

            %Loop through each layer of the part tree, starting at the
            %bottom layer
            for layer = obj.part_tree.num_layers:-1:1
                %Loop through each part in the current layer
                for part_num = 1:size(obj.part_tree.tree_matrix{layer}, 2)
                    part_index = obj.part_tree.tree_matrix{layer}(part_num);

                                        
                    


                    %Gets all of the children of the current part
                    children_indices = obj.part_tree.find_children(part_index);

                    %Indices of all parts to apply current transformation
                    %to
                    transform_indices = [part_index, children_indices];



                    part_rotation = obj.part_rotations(part_index, :);
                    part_translation = obj.part_locations(part_index, :);
                    pivot_point = obj.pivot_points(part_index, :);

                    %Loops through every child an applies the same
                    %transformations
                    for child_index = transform_indices
                        if child_index == 1
                            disp("");
                        end

                        %Rotates part according to predefined rotation
                        current_part = obj.transformed_parts{child_index}.rotate_part(part_rotation, pivot_point);

                        %Translates part according to predefined translation
                        current_part = current_part.translate_part(part_translation);

                            
                        %Applies transformationss
                        obj.transformed_parts{child_index} = current_part;


                    end

                end

            end

            total_COM = zeros(1, 3);

            

            for i = 1:obj.num_parts
                [new_part_mass, new_part_COM] = obj.transformed_parts{i}.find_COM();

                total_COM = total_COM + new_part_mass*new_part_COM;
            end

            obj.COM = total_COM/obj.mass;

            disp(obj.mass);

            disp(obj.COM);

        end

        function generate_surface(obj)
            bounding_box = [-1.1,-1,-1;2,1,1];
            interval = 0.04;


            [X,Y,Z] = meshgrid(bounding_box(1, 1):interval:bounding_box(2, 1), ...
                bounding_box(1, 2):interval:bounding_box(2, 1), ...
                bounding_box(1, 3):interval:bounding_box(2, 3));


            surface_values = zeros([size(X), size(obj.transformed_parts, 1)]);

            tic

            for i = 1:size(obj.transformed_parts, 1)
                disp(i);
                surface_values(:, :, :, i) = ...
                   obj.transformed_parts{i}.surface_mesh.generate_surface(X, Y, Z, interval);
            end

            surface_points = min(surface_values, [], 4);

            [face_nodes, node_coords] = isosurface(X, Y, Z, surface_points, 0.01);

            patch('Faces', face_nodes,'Vertices', node_coords, 'FaceColor','red');

            writeMeshtoObj(node_coords, face_nodes, "aircraftModel");

            xlim([-1, 1]);
            ylim([-1, 1]);
            zlim([-1, 1]);

            %[p, t] = distmeshsurface( SDF, FH, 0.02, bounding_box);

            toc

            

        end

        function show(obj, axes, type)

            if(nargin == 3)
                hold(axes,'on')
                
                
                for i = 1:size(obj.transformed_parts, 1)
                    obj.transformed_parts{i}.show(type);
                end
            end

            if(nargin == 2)
                hold(axes,'on')
                
                for i = 1:size(obj.transformed_parts, 1)
                    obj.transformed_parts{i}.show_facets(axes);

                end

                axes.XLabel.String = "X";
                axes.YLabel.String = "Y";
                axes.ZLabel.String = "Z";
            elseif(nargin == 1)
                hold on;
                
                for i = 1:size(obj.transformed_parts, 2)
                    obj.transformed_parts{i}.show_facets();
                end
            end

            

            
        end

    end

end