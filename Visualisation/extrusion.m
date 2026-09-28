%Name:        Isaac Nakone, Alex Welke
%Date:        16/07/2024 - 01/08/2024
%Description: This is an extrusion class. It's basically a shape class but 
%             it can only do extrusions. I have also refactored a lot of
%             the shape class here to for example include the new profile
%             class as a property i.e. profile vector.


classdef extrusion

    properties
        %List of profiles that make up the extrusion
        profiles = {};
        %Coordinates of each node in 3d space
        nodes2coords = [];
        nodes2quats = [];

        %Nodes in every tetrahedron
        tetras2nodes = [];

        %Nodes in every exterior triangle
        ext_facets2nodes = [];

        %Adjacency matrix describing connections in extrusion
        adjacency_matrix = [];

        %Surface mesh of the part
        surface_mesh  = [];

        %Extra mass not part of the extrusion model
        additional_mass = [];

        %Location of additional mass
        extra_mass_locations = [];
        extra_mass_quats = [];

        %Density of part
        part_density = 0;

    end

    methods

        function obj = extrusion(base_profile, extrude_marks, xy_pos_vals, scale_vals, density, additionalMass, massPos)

            %NOTE: The connection_profile is a vector which has as its first
            %      dimension  the first dimension of base_profile.nodes2coords.
            %      It is a vector of "z"-coordinates of each point of the
            %      base_profile so that the base_profile conforms to a
            %      curved shape.
            % extrude_marks - 1 x n matrix representing points at which the
            % thickness of the profile is defined in side_profile_matrix
            % side_profile_matrix - n x 2 matrix representing the vertical 
            % coordinate of the desired upper and lower side of the
            % extrusion at each horizontal coordinate defined in
            % extrude_marks. This allows variable thickness along the
            % extrusion
            % sweep - the offset between the centroid of the profile at the 
            % start and end of the extrusion, as measured along the x 
            % coordinate of the profile coordinate system

            obj.part_density = density;

            if (nargin == 7) 
                obj.additional_mass = additionalMass;
                obj.extra_mass_locations = massPos;
            end


            num_profiles = size(extrude_marks, 2);

            obj.profiles = cell(num_profiles, 1);

            %Set each profile to the base profile scaled and translated:
            for profile_index = 1:num_profiles


                %Now scale and translate:
                extrude_amount     = extrude_marks(profile_index);

                scale_factor       = scale_vals(profile_index);

                translate_planar = xy_pos_vals(profile_index, :);

                obj.profiles{profile_index} = base_profile.scale_planar(scale_factor);
                obj.profiles{profile_index} = obj.profiles{profile_index}.translate_planar(translate_planar);
                obj.profiles{profile_index} = obj.profiles{profile_index}.translate(extrude_amount);

                %Construct the nodes2coords matrix based on the profiles:
                obj.nodes2coords = [obj.nodes2coords; obj.profiles{profile_index}.nodes2coords ];


            end


            %Gets adjacency matrix of base profile
            A = obj.profiles{1}.node_adjacency_matrix;

            Ablk = generateExtrusionAdj(A, num_profiles);
            obj.adjacency_matrix = Ablk;


            %Finds the size of the block matrix
            blk_sz = size(Ablk, 1);


            %Calculates the number of tetrahedrons in the mesh and preallocates a
            %matrix to store the data
            num_tetras = trace(full(A)^3)*0.5*(num_profiles-1);
            obj.tetras2nodes = zeros(num_tetras, 4);


            num_triangles = trace(full(Ablk)^3)/6;
            obj.ext_facets2nodes = zeros(num_triangles, 3);

            
            %Gets the locations of all of the non zero elements in the adjacency matrix (edges)
            [edgeRows, edgeCols, ~] = find(triu(Ablk));

            i = 1;
            j = 1;
            
            %Loops through every connection in the adjacency matrix
            for edgeIndex = 1:size(edgeRows, 1)
                row = edgeRows(edgeIndex);
                col = edgeCols(edgeIndex);

                %Checks all elements directly to the right and directly below the
                %current element
                for point3 = (col+1):blk_sz
                    %Checks whether the current point index connects to both of
                    %the other points (the indices of the first points are the 
                    %row number and the column number that was arrived at in the prevous
                    % embedded for loop)
                    if(Ablk(row, point3) + Ablk(point3, col) == 2)
                        %Stores the number of tetrahedrons that the current triangle is
                        %a part of
                        num_points4 = 0;
                        %Checks all points with an index greater than the index
                        %of the third point
                        for point4 = (point3+1):blk_sz
                            %Checks whether the current point index connects to
                            %all of the three other points
                            if(Ablk(row, point4) + Ablk(point4, col) + Ablk(point4, point3) == 3)
                                obj.tetras2nodes(i, :) = [row, col, point3, point4];
                                i = i + 1;
                                num_points4 = num_points4 + 1;
                            end
                        end


                        %Checks whether the current triangle can already be
                        %ruled out as an interior triangle
                        if(num_points4 < 2)

                            %Searches through the rest of the possible nodes to find any
                            %potential tetrahedrons that could be formed, even through they
                            %are not recorded to avoid double-ups
                            for point4 = 1:(point3-1)
                                %Checks whether the current point index connects to
                                %all of the three other points
                                if(Ablk(row, point4) + Ablk(point4, col) + Ablk(point4, point3) == 3)
                                    num_points4 = num_points4 + 1;
                                end
                            end



                            %Checks whether the current triangle is visible, or whether it
                            %is covered up on either side by a tetrahedron
                            if(num_points4 < 2)
                                 obj.ext_facets2nodes(j, :) = [row, col, point3];
                                 j = j + 1;
                            end

                        end
                    end
                end
            end


            %Cuts the trailing zeros off of the matrix
            num_triangles = size(find(obj.ext_facets2nodes(:, 3)), 1);
            obj.ext_facets2nodes = obj.ext_facets2nodes(1:num_triangles, :);

            obj.surface_mesh = extrusion_surface(base_profile, xy_pos_vals, extrude_marks, scale_vals);
            
        end

        function [total_COM, total_mass] = find_COM(obj)

            %Finds the mass and COM of the part
            total_volume = 0;

            total_COM = zeros(1, 3);


            for i = 1:size(obj.tetras2nodes, 1)
                nodeIndices = obj.tetras2nodes(i, :);


                nodeCoords = [obj.nodes2coords(nodeIndices(1), :);
                                obj.nodes2coords(nodeIndices(2), :);
                                obj.nodes2coords(nodeIndices(3), :);
                                obj.nodes2coords(nodeIndices(4), :)];

                % Volume of the tetrahedron (using the determinant of the Jacobian)
                J = [
                    nodeCoords(2, 1) - nodeCoords(1, 1), nodeCoords(3, 1) - ...
                    nodeCoords(1, 1), nodeCoords(4, 1) - nodeCoords(1, 1);
                    nodeCoords(2, 2) - nodeCoords(1, 2), nodeCoords(3, 2) - ...
                    nodeCoords(1, 2), nodeCoords(4, 2) - nodeCoords(1, 2);
                    nodeCoords(2, 3) - nodeCoords(1, 3), nodeCoords(3, 3) - ...
                    nodeCoords(1, 3), nodeCoords(4, 3) - nodeCoords(1, 3)
                ];

                %COM of the tetrahedron
                centroid = mean(nodeCoords, 1);

                volume = (1/6)*abs(det(J));

                total_volume = total_volume + volume;

                total_COM = total_COM + centroid*volume;


            end

            total_mass = total_volume * obj.part_density;
            total_COM = total_COM/total_volume;

            total_COM = total_COM*total_mass + sum(obj.additional_mass.*obj.extra_mass_locations);

            total_mass = total_mass + sum(obj.additional_mass);

        end

        function obj = translate_part(obj, translation)
            
            obj.nodes2coords = ...
            obj.nodes2coords + translation;

            if(~isempty(obj.additional_mass))
                obj.extra_mass_locations = ...
                obj.extra_mass_locations + translation;
            end

            obj.surface_mesh.transformations = [obj.surface_mesh.transformations; 
                                                translation, 0, 0];

        end

        function obj = rotate_part(obj, rotation_quat, pivot_point)
            %Gets rotation angle and axis from info
            rotation_angle = rotation_quat(4);
            axis_unit_vector = rotation_quat(1:3);

            %Translates part so that it rotates around its pivot point
            temp_nodes2coords = obj.nodes2coords - pivot_point;


            obj.nodes2quats = quaternion(zeros(size(temp_nodes2coords,1),1), ...
                temp_nodes2coords(:,1), temp_nodes2coords(:,2), ...
                temp_nodes2coords(:,3));

            rotation_quaternion = quaternion(cosd(rotation_angle/2), ...
                axis_unit_vector(1)*sind(rotation_angle/2), ...
                axis_unit_vector(2)*sind(rotation_angle/2), ...
                axis_unit_vector(3)*sind(rotation_angle/2));

            obj.nodes2quats = ...
                conj(rotation_quaternion)*obj.nodes2quats...
                *rotation_quaternion;

            temp_nodes2coords = ...
                compact(obj.nodes2quats);

            %Returns part to previous position
            obj.nodes2coords = ...
                temp_nodes2coords(:,2:4) + pivot_point;

            %Also rotates extra masses in the part
            if(~isempty(obj.additional_mass))
                temp_mass_locations = obj.extra_mass_locations - pivot_point;

                temp_mass_quats = quaternion(zeros(size(temp_mass_locations,1),1), ...
                temp_mass_locations(:,1), temp_mass_locations(:,2), ...
                temp_mass_locations(:,3));

                temp_mass_quats = ...
                    conj(rotation_quaternion)*temp_mass_quats...
                    *rotation_quaternion;

                temp_mass_locations = ...
                compact(temp_mass_quats);

                obj.extra_mass_locations = ...
                temp_mass_locations(:,2:4) + pivot_point;
            end

            %Adds transformation to surface mesh
            obj.surface_mesh.transformations = [obj.surface_mesh.transformations; 
                                                axis_unit_vector, rotation_angle, 1];

        end


        function show(obj,type)
            
            if strcmp(type, "elems")
                hold on;
    
                facets2lodes = [1 2 3; 2 3 4; 1 3 4; 1 2 4];
                for tetra_index = 1:size(obj.tetras2nodes,1)
    
                    for facet_index = 1:4
                        
                        
                        lode_indices = facets2lodes(facet_index, :);
                        node_indices  = obj.tetras2nodes(tetra_index,lode_indices);
                        coords = obj.nodes2coords(node_indices,:);
                        fill3(coords(:,1),coords(:,2),coords(:,3),'b','faceAlpha',0.3,'EdgeColor','y');
                        xlim([-2,2]);
                        ylim([-2,2]);
                        zlim([-2,2]);
                        view(3)
                        xlabel('x');
                        ylabel('y');
                        zlabel('z');
                        axis square

                    end
        
                end
    
                set(gca,'color','k');

            elseif strcmp(type, "facets")

                hold on;
                
                for e = 1:size(obj.tetras2nodes,1)
                    nodes = obj.tetras2nodes(e,:);
                    coords = obj.nodes2coords(nodes,:);
                
                    local_facets2nodes = [1 2 3;
                                            1 2 4;
                                            2 3 4;
                                            1 3 4];
                
                    for f = 1:4
                        facet_nodes = local_facets2nodes(f,:);
                        facet_coords = coords(facet_nodes,:);
                    
                    
                        fill3(facet_coords(:,1),facet_coords(:,2), facet_coords(:,3),'r', 'FaceAlpha',0.5,'EdgeColor','y');
                        view(3)
                
                    end

                end
                
                set(gca,'color','k')
                axis equal
            end

         end

         function show_facets(obj, axes)
             
            if (nargin == 2)
                 patch(axes, 'Faces', obj.ext_facets2nodes, 'Vertices', obj.nodes2coords, ...
                 'FaceColor', 'r','EdgeColor','y');
                
                set(axes, 'color','k');


                axes.XLabel.String = "X";
                axes.YLabel.String = "Y";
                axes.ZLabel.String = "Z";
            elseif (nargin == 1)
                axes = gca;

                patch('Faces', obj.ext_facets2nodes, 'Vertices', obj.nodes2coords, ...
                 'FaceColor', 'r','EdgeColor','y');
                
                set(axes, 'color','k');


                axes.XLabel.String = "X";
                axes.YLabel.String = "Y";
                axes.ZLabel.String = "Z";
            end
         end

    end

end