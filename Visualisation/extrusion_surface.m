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
        poly_y_pos = [];
        poly_scale = [];

        %nx4 matrix. First 4 columns describe each transformation and the
        %last column describes if it is a translation (0) or a rotation (1)
        transformations = [];

    end
    methods

    
      
        function obj = extrusion_surface(base_profile, extrude_marks, side_profile_matrix, sweep, resolution)
            outer_profile = base_profile.change_inset([]);

            obj.x_sample = extrude_marks;

            %Find the y-size of the base profile:
            y_extent_base_profile = max(outer_profile.vertices2coords(:,2))-...
                                    min(outer_profile.vertices2coords(:,2));

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


            %Finds implicit curve of surface

            %Finds polynomials fit to extrude marks

            y_positions = [extrude_marks', mean(side_profile_matrix, 2)];

            %y_positions = [2*y_positions(1, :) - y_positions(2, :);...
                %y_positions; 2*y_positions(end, :) - y_positions(end-1, :)];
            

            scale_amounts = (side_profile_matrix(:, 1) - ...
                side_profile_matrix(:, 2))/y_extent_base_profile;

            scale_amounts = [extrude_marks', scale_amounts];
            

            obj.poly_y_pos = vander(y_positions(:, 1)')\y_positions(:, 2);
            

            % x_val = linspace(0, 0.3, 20);
            % y_pos = polyval(obj.poly_y_pos, x_val);
            % 
            % plot(x_val, y_pos, 'r-', y_positions(:, 1), y_positions(:, 2),'ko');
            % axis equal;

            obj.poly_scale = vander(scale_amounts(:, 1)')\scale_amounts(:, 2);

            %cla;
            

            %y_scale = polyval(obj.poly_scale, x_val);

            % plot(x_val, y_scale);
            % axis equal;


        end


        function eqVals = generate_surface(obj, X, Y, Z)

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

            poly_upper = obj.base_profile.poly_upper;
            poly_lower = obj.base_profile.poly_lower;


            % x_val = linspace(-0.5, 0.5, 10);
            % 
            % upper_vertices = [x_val', polyval(poly_upper, x_val)'];
            % 
            % lower_vertices = [flip(x_val, 2)', polyval(poly_lower, flip(x_val, 2))'];
            % 
            % plot(upper_vertices(:, 1), upper_vertices(:, 2),'r-');
            % hold on;
            % plot(lower_vertices(:, 1), lower_vertices(:, 2),'r-');
            % hold off;
            % axis equal;

            dir = obj.base_profile.translate_direction;


            X = (X-(obj.sweep./obj.extrusion_length).*...
                (dir*Z-obj.x_sample(1)))./polyval(obj.poly_scale, dir*Z);

            Y = (Y-polyval(obj.poly_y_pos, dir.*Z))./...
                polyval(obj.poly_scale, dir.*Z);


            profileUpper = -polyval(poly_upper, X)+Y;
            profileLower = polyval(poly_lower, X)-Y;


            beginFace = dir.*(dir.*obj.x_sample(1)-Z);
            endFace = dir.*(Z-dir.*obj.x_sample(end));

            eqVals = max(cat(4, profileUpper, profileLower, beginFace, endFace), [], 4);


            % isosurface(Xi, Yi, Zi, eqVals, 0.01);
            % hold off;

            % xlabel('x');
            % ylabel('y');
            % zlabel('z');

            % xlim([-0.5,1]);
            % ylim([-0.75,0.75]);
            % zlim([-0.75,0.75]);


        end

    end
    
end