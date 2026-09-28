%Name: Alex
%Date: 22/8/24
%Description: Generates a set of points that defines an airfoil based on a
%set of parameters, places results into a .mat file with the given name of
%the airfoil

function airfoil_points = generate_NACA_airfoil(max_camber_percent, max_camber_pos_decile, ...
    max_thickness_percent, airfoil_height, num_points)
    max_camber = max_camber_percent/100;
    max_camber_pos = max_camber_pos_decile/10;
    max_thickness = max_thickness_percent/100;

    if(max_camber_pos ~= 0)
        x_values_1 = linspace(0, max_camber_pos, ceil(num_points/2));
        x_values_2 = linspace(max_camber_pos, 1, floor(num_points/2));
    else
        x_values_1 = linspace(0, 1, num_points);
        x_values_2 = [];
    end

   
    x_values = [x_values_1, x_values_2(2:end)];

    x_size = size(x_values, 2);

    camber_line = zeros(x_size, 1);

    camber_slopes = zeros(x_size, 1);
    %Generates camber line
    for x_num = 1:x_size
        x_pos = x_values(x_num);
        if(x_pos < max_camber_pos)
            camber_line(x_num) = max_camber/(max_camber_pos^2) * ...
            (2*max_camber_pos*x_pos-x_pos^2);

            camber_slopes(x_num) = 2*max_camber/(max_camber_pos^2) * (max_camber_pos-x_pos);
        else
            camber_line(x_num) = max_camber/((1-max_camber_pos)^2) * ...
            ((1-2*max_camber_pos) + 2*max_camber_pos*x_pos-x_pos^2);

            camber_slopes(x_num) = 2*max_camber/((1-max_camber_pos)^2) * (max_camber_pos-x_pos);
        end
    end

    upper_points = zeros(x_size, 2);
    lower_points = zeros(x_size, 2);

    %Adds the thickness to the camber
    for x_num = 1:x_size
        x_pos = x_values(x_num);
        y_pos = camber_line(x_num);
        theta = atan(camber_slopes(x_num));

        thickness = 5*max_thickness*(0.2969*sqrt(x_pos) - ...
            0.1260*x_pos-0.3516*x_pos^2+0.2843*x_pos^3 ...
            -0.1036*x_pos^4);
    
        %Adds upper point
        upper_points(x_num, 1) = x_pos - thickness * sin(theta);
        upper_points(x_num, 2) = y_pos + thickness * cos(theta);
    
        %Adds lower point
        lower_points(x_num, 1) = x_pos + thickness*sin(theta);
        lower_points(x_num, 2) = y_pos - thickness*cos(theta);

    end

    %Combines points into single list
    airfoil_points = [upper_points; flip(lower_points(2:end-1, :), 1)];


    %Centres the profile around (0, 0)
    avg_y = (max(airfoil_points(:, 2)) + min(airfoil_points(:, 2)))/2;

    airfoil_points = airfoil_points - repmat([0.5, avg_y], [x_size*2-2, 1]);

    airfoil_y_extent = max(airfoil_points(:, 2)) - min(airfoil_points(:, 2));

    %Scales the airfoil correctly
    airfoil_points = airfoil_points* (airfoil_height/airfoil_y_extent);

    
    % plot(airfoil_points(:, 1), airfoil_points(:, 2), 'r-')
    % hold on;
    
end