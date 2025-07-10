%Name:        Alex Welke
%Date:        16/07/2024 - 01/08/2024
%Description: This is a profile class designed to take in planar 
%             polygonal points and a vector normal
%             which is perpendicular to the plane.

classdef extrusion_surface
    properties
        profiles = [];
        x_sample = [];
        base_profile = [];
        nodes2coords = [];
        nodes2quats = [];

        sweep = 0;
        extrusion_length = 0;
        y_pos_points = [];
        scale_points = [];

        %nx4 matrix. First 4 columns describe each transformation and the
        %last column describes if it is a translation (0) or a rotation (1)
        transformations = [];

    end
    methods

    
      
        function obj = extrusion_surface(base_profile, extrude_marks, side_profile_matrix, sweep)
            outer_profile = base_profile.change_inset([]);

            obj.x_sample = extrude_marks;

            %Find the y-size of the base profile:
            y_extent_base_profile = max(outer_profile.vertex_coords(:,2))-...
                                    min(outer_profile.vertex_coords(:,2));

            %Find the scaling factor for the first profile:
            initial_scale_factor       = (side_profile_matrix(1,1)-side_profile_matrix(1,2))/y_extent_base_profile;
            initial_translate_planar_y = mean(side_profile_matrix(1,:));

            obj.profiles = outer_profile;
            obj.base_profile = outer_profile;

            obj.profiles(1) = obj.profiles(1).scale_planar(initial_scale_factor);
            obj.profiles(1) = obj.profiles(1).translate_planar( [0,initial_translate_planar_y] );

            extrusion_length = extrude_marks(end)-extrude_marks(1);
            obj.extrusion_length = extrusion_length;

            obj.nodes2coords = [obj.profiles(1).nodes2coords];

            obj.sweep = sweep;

            y_positions = [extrude_marks', mean(side_profile_matrix, 2)];

            scale_amounts = (side_profile_matrix(:, 1) - ...
                side_profile_matrix(:, 2))/y_extent_base_profile;

            scale_amounts = [extrude_marks', scale_amounts];
            

            obj.y_pos_points = y_positions;
            obj.scale_points = scale_amounts;

        end


        function eqVals = generate_surface(obj, X, Y, Z, interval)

            Xi = X;
            Yi = Y;
            Zi = Z;

            for i = size(obj.transformations, 1):-1:1


                %Checks if the transformation specified by this row in the matrix
                %is a rotation or translation. Last element of the row is a
                %0=translation. 1=rotation.
                if obj.transformations(i, 5) == 0
                    X = X - obj.transformations(i, 1);
                    Y = Y - obj.transformations(i, 2);
                    Z = Z - obj.transformations(i, 3);
                else
                    
                    rotation_angle = obj.transformations(i, 4);
                    axis_unit_vector = obj.transformations(i, 1:3);
                    
                    rotation_quat = quaternion(cosd(rotation_angle/2), ...
                    axis_unit_vector(1)*sind(rotation_angle/2), ...
                    axis_unit_vector(2)*sind(rotation_angle/2), ...
                    axis_unit_vector(3)*sind(rotation_angle/2));
                    
                    [~, X, Y, Z] = parts(rotation_quat*...
                        quaternion(zeros(size(X)), X, Y, Z) *...
                        conj(rotation_quat));




                    % transform_x = @(x, y, z) obj.quats2values(rotation_quat*...
                    %     quaternion(zeros(size(x)), prev_transform_x(x, y, z), ...
                    %     prev_transform_y(x, y, z), prev_transform_z(x, y, z))*...
                    %     conj(rotation_quat), 2);
                    % transform_y = @(x, y, z) obj.quats2values(rotation_quat*...
                    %     quaternion(zeros(size(y)), prev_transform_x(x, y, z), ...
                    %     prev_transform_y(x, y, z), prev_transform_z(x, y, z))*...
                    %     conj(rotation_quat), 3);
                    % transform_z = @(x, y, z) obj.quats2values(rotation_quat*...
                    %     quaternion(zeros(size(z)), prev_transform_x(x, y, z), ...
                    %     prev_transform_y(x, y, z), prev_transform_z(x, y, z))*...
                    %     conj(rotation_quat), 4);

                end
            end


            %Precalculated table of values for scaling and vertical
            %translations along extrusion
            yPosVals = linearInterp(obj.y_pos_points, interval, Z);
            yScaleVals = linearInterp(obj.scale_points, interval, Z);


            X = (X-(obj.sweep./obj.extrusion_length).*...
                (Z-obj.x_sample(1)))./yScaleVals;

            Y = (Y-yPosVals)./yScaleVals;

            %Calculate SDF of profile
            profileSDF = generate_SDF(obj.base_profile.vertex_coords, X, Y, interval);

            beginFace = min(obj.x_sample, [], 2)-Z;
            endFace = Z-max(obj.x_sample, [], 2);

            eqVals = max(cat(4, profileSDF, beginFace, endFace), [], 4);


            % [face_nodes, node_coords] = isosurface(Xi, Yi, Zi, eqVals, 0.01);
            % 
            % patch('Faces', face_nodes,'Vertices', node_coords, 'FaceColor','red');
            % 
            % xlabel('x');
            % ylabel('y');
            % zlabel('z');
            % 
            % xlim([-0.5,1.5]);
            % ylim([-0.75,0.75]);
            % zlim([-0.75,0.75]);


       end

    end
    
end