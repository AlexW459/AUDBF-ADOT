%Name:        Alex Welke
%Date:        6/07/2025
%Description: This function approximates the value of a function 
% (defined by a set of points) for a range of x-values

function y_pos_table = linearInterp(points, int_start, interval, int_end)

    y_pos_table = interp1(points(:, 1), points(:, 2), ...
        min(points(:, 1)):interval:max(points(:, 1)), 'linear');

    prevFiller = floor((min(points(:, 1))-int_start)/interval);
    follFiller = floor((int_end-max(points(:, 1)))/interval);

    y_pos_table = [ones(1, prevFiller)*points(1, 2),...
        y_pos_table, ones(1, follFiller)*points(end, 2)];

end
