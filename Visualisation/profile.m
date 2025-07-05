%Name:        Isaac Nakone, Alex Welke
%Date:        16/07/2024 - 01/08/2024
%Description: This is a profile class designed to take in planar 
%             polygonal points and a vector normal
%             which is perpendicular to the plane.


classdef profile

    properties 

        translate_direction = [];      %Determines whether the extrusion extrudes forwards or backwards. Equal to 1 or -1
        inset = [];
        vertex_coords = []; %The 2d coordinates of each vertex (outer).
        inset_vertex_coords = [];  %The 2d coordinates of each vertex (if there's inset).
        nodes2coords = [];    %The 3d coordinates of each node (a 3d vertex).
        polygon = [];         %MATLAB polyshape object representing the shape to be extruded
        centroid = [];        % centroid of the shape, saved as a set of coordinates
        triangles = [];   %MATLAB triangulation object, used as an intermediate step to calculate node_adjacency_matrix 
        adjacency_matrix = [];%deprecated - never used
        node_adjacency_matrix = [];%adjacency matrix of every vertex/node, 
                                   %representing which other vertices/nodes
                                   %they connect to via edges
        outer_adjacency_matrix = []; %Adjacency matrix of non-inset points only

    end

    methods
        function obj = profile(profile_vertex_coords, direction, inset)
            % constructor
            % vertex coords - n by 2 matrix, each row is point in 2d
            % space, each point being connected to the point before and
            % after in the matrix

            obj.vertex_coords = profile_vertex_coords;

            obj.centroid = [(max(obj.vertex_coords(1:end-1, 1))+min(obj.vertex_coords(1:end-1, 1)))/2,...
                            (max(obj.vertex_coords(1:end-1, 2))+min(obj.vertex_coords(1:end-1, 2)))/2];
            obj.translate_direction = direction;
            
            obj.inset = inset;

            obj.nodes2coords = [obj.vertex_coords, zeros(size(obj.vertex_coords, 1), 1)];

            obj = obj.generate_inset();

            % plot(obj.vertex_coords(:, 1), obj.vertex_coords(:, 2), ...
            %     obj.inset_vertex_coords(:, 1), obj.inset_vertex_coords(:, 2));
            % axis equal;


            num_outer_points = size(obj.vertex_coords, 1);
            obj.outer_adjacency_matrix = [zeros(num_outer_points-1, 1), eye(num_outer_points-1); 
                                          zeros(1, num_outer_points)];
            obj.outer_adjacency_matrix(1, num_outer_points) = 1;
            obj.outer_adjacency_matrix = obj.outer_adjacency_matrix+obj.outer_adjacency_matrix';


            %plot(obj.inset_vertices2coords(:, 1), obj.inset_vertices2coords(:, 2), 'r',...
            %obj.vertices2coords(:, 1), obj.vertices2coords(:, 2), 'b');

        end

        function obj = generate_inset(obj)
            if(isempty(obj.inset))
                obj.inset_vertex_coords = [];

                obj.polygon = polyshape(obj.vertex_coords(:, 1),obj.vertex_coords(:, 2));
                %Finds triangulation from points
                obj.triangles = triangulation(obj.polygon);

                obj.nodes2coords = [obj.triangles.Points, obj.nodes2coords(1, 3)*ones(size(obj.triangles.Points, 1), 1)];

                %Gets list of edges in triangulation
                edgeList = edges(obj.triangles);

                G = graph(edgeList(:,1),edgeList(:,2));

                %Gets the adjacency matrix from the graph object
                obj.node_adjacency_matrix = adjacency(G);
                return;
            end

            rot_mat = [0, -1; 1, 0]; % 90 degree rotation

            perp_vectors = zeros(size(obj.vertex_coords, 1), 2); % initialises matrix for perpendicular 
            inset_vectors = zeros(size(obj.vertex_coords, 1), 2);

            perp_vectors(end, :) = (rot_mat*(obj.vertex_coords(end, :) - obj.vertex_coords(1, :))')';

            perp_vectors(1:end-1, :) = (rot_mat*(obj.vertex_coords(1:end-1, :)' - obj.vertex_coords(2:end, :)'))';

            perp_vectors = perp_vectors./sqrt(sum(perp_vectors.^2,2));
            
            inset_vectors(1, :) = 0.5*(perp_vectors(end, :) + perp_vectors(1, :));
            inset_vectors(2:end, :) = 0.5*(perp_vectors(1:end-1, :) + perp_vectors(2:end, :));
            inset_vectors = inset_vectors./sqrt(sum(inset_vectors.^2,2));


            obj.inset_vertex_coords = obj.vertex_coords + inset_vectors*obj.inset;


            for i = 1:size(obj.inset_vertex_coords, 1)
                for j = 1:size(obj.vertex_coords, 1)
                    diff = obj.vertex_coords(j, :) - obj.inset_vertex_coords(i, :);
                    dist = norm(diff);

                    %Checks whether an inner node is too close to an
                    %outer node, indicating an error
                    % makes the inset very small if the profile is too
                    % thin at that point
                    if dist < obj.inset && j ~= i

                        obj.inset_vertex_coords(i, :) = obj.inset_vertex_coords(i, :) -...
                        inset_vectors(i, :)*(obj.inset-10^(-4.5));

                        break;
                    end
                end
            end

            
            obj.polygon = polyshape({obj.vertex_coords(:, 1), obj.inset_vertex_coords(:, 1)},...
                {obj.vertex_coords(:, 2), obj.inset_vertex_coords(:, 2)});
            % 
            % plot(obj.polygon);
            axis equal;

            %Finds triangulation from points
            obj.triangles = triangulation(obj.polygon);

            obj.nodes2coords = [obj.triangles.Points, obj.nodes2coords(1, 3)*ones(size(obj.triangles.Points, 1), 1)];


            %Gets list of edges in triangulation
            edgeList = edges(obj.triangles);

            G = graph(edgeList(:,1),edgeList(:,2));

            %Gets the adjacency matrix from the graph object
            obj.node_adjacency_matrix = adjacency(G);

        end

        function obj = change_inset(obj, new_inset)
            obj.inset = new_inset;

            obj.generate_inset();

        end

        function obj = change_direction(obj, new_direction)
            obj.translate_direction = new_direction;
        end

        % sets the object to have new vertex location in 2D space equal 
        % to the old vertex location translated by translation
        % translation - a 2D vector representing the amount to be
        % translated by
        function obj = translate_planar(obj, translation)

            obj.vertex_coords = obj.vertex_coords + translation;

            if(~isempty(obj.inset))
                obj.inset_vertex_coords = obj.inset_vertex_coords + translation;
                obj.polygon = polyshape({obj.vertex_coords(:, 1), obj.inset_vertex_coords(:, 1)},...
                {obj.vertex_coords(:, 2), obj.inset_vertex_coords(:, 2)});
            else
                obj.polygon = polyshape(obj.vertex_coords(:, 1), obj.vertex_coords(:, 2));
            end

            %Recalculates centroid
            obj.centroid = [(max(obj.vertex_coords(:, 1))+min(obj.vertex_coords(:, 1)))/2,...
                            (max(obj.vertex_coords(:, 2))+min(obj.vertex_coords(:, 2)))/2];

            %Finds triangulation from points
            obj.triangles = triangulation(obj.polygon);

            obj.nodes2coords = [obj.triangles.Points, obj.nodes2coords(1, 3)*ones(size(obj.triangles.Points, 1), 1)];
        end


        function obj = scale_planar(obj, scale_factor)
            % dilates the shape by a factor of scale_factor about the
            % origin of the coordinate system
            % scale_factor - the factor to scale the shape by about the
            % origin of the coordinate system
            temp_vertices = obj.vertex_coords - obj.centroid;
            temp_vertices = temp_vertices*scale_factor;
            obj.vertex_coords = temp_vertices + obj.centroid;

            obj = obj.generate_inset();
        end


        function obj = rotate_planar(obj, angle)
            temp_vertices = obj.vertex_coords - obj.centroid;
            temp_inset_vertices = obj.inset_vertex_coords - obj.centroid;

            rotation_matrix = [cosd(angle), -sind(angle);
                               sind(angle), cosd(angle)];

            temp_vertices = temp_vertices.*repmat(rotation_matrix, ...
                [1, size(temp_vertices, 1)]);

            temp_inset_vertices = temp_inset_vertices.*repmat(rotation_matrix, ...
                [1, size(temp_inset_vertices, 1)]);

            obj.vertex_coords = temp_vertices + obj.centroid;
            obj.inset_vertex_coords = temp_inset_vertices + obj.centroid;

            obj.nodes2coords = [obj.triangles.Points, obj.nodes2coords(1, 3)*ones(size(obj.triangles.Points, 1), 1)];
        end


        function obj = translate(obj, translate_amount)

            obj.nodes2coords = obj.nodes2coords + [0, 0, 1]*translate_amount*obj.translate_direction;

        end


        function plot_profile(obj)
            hold on;
            for facet_index = 1:size(obj.triangles.ConnectivityList,1)
                nodes = obj.triangles.ConnectivityList(facet_index,:);
                coords = obj.nodes2coords(nodes,:);
                
                fill3(coords(:,1), coords(:,2), coords(:,3),'r');

            end

            axis equal
            view(3)
            set(gca,'color','k');

        end

    end

end
