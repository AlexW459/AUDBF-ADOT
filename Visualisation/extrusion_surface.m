%Name:        Alex Welke
%Date:        16/07/2024 - 01/08/2024
%Description: This is a profile class designed to take in planar 
%             polygonal points and a vector normal
%             which is perpendicular to the plane.

classdef extrusion_surface
    properties

        base_profile = [];

        z_sample;
        xy_pos_points;
        scale_vals = [];

        %nx4 matrix. First 4 columns describe each transformation and the
        %last column describes if it is a translation (0) or a rotation (1)
        transformations = [];

    end
    methods

    
        function obj = extrusion_surface(base_profile_, xy_pos, extrude_marks, scale_points)
            
            obj.z_sample = extrude_marks;
            obj.base_profile = base_profile_;

            obj.xy_pos_points = xy_pos;
            obj.scale_vals = scale_points;
            obj.z_sample = extrude_marks;


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

                end
            end


            %Precalculated table of values for scaling and vertical
            %translations along extrusion
            xPosVals = linearInterp([obj.z_sample', obj.xy_pos_points(:, 1)], interval, Z);
            yPosVals = linearInterp([obj.z_sample', obj.xy_pos_points(:, 2)], interval, Z);
            scaleVals = linearInterp([obj.z_sample', obj.scale_vals'], interval, Z);

            X = (X-xPosVals)./scaleVals;

            Y = (Y-yPosVals)./scaleVals;

            %Calculate SDF of profile
            profileSDF = generate_SDF(obj.base_profile.vertex_coords, X, Y, interval);

            beginZ = min(obj.z_sample, [], 2);
            endZ = max(obj.z_sample, [], 2);

            beginFace = (beginZ-Z)./sqrt(1+beginZ^2);
            endFace = (Z-endZ)./sqrt(1+endZ^2);

            eqVals = max(cat(4, profileSDF, beginFace, endFace), [], 4);


            % % [face_nodes, node_coords] = isosurface(Xi, Yi, Zi, eqVals, 0.01);
            % % clf;
            % % patch('Faces', face_nodes,'Vertices', node_coords, 'FaceColor','red');
            % % 
            % % xlabel('x');
            % % ylabel('y');
            % % zlabel('z');
            % % 
            % % xlim([-0.5,1.5]);
            % % ylim([-0.75,0.75]);
            % % zlim([-0.75,0.75]);



       end

    end
    
end