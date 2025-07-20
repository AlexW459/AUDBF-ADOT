%Name:        Alex Welke
%Date:        6/07/2025
%Description: This function approximates the value of a function 
% (defined by a set of points) for a range of x-values

function yPosVals = linearInterp(points, interval, X)
    intStartX = min(X, [],"all");
    intEndX = max(X, [], "all");
    xPosTable = intStartX:interval:intEndX;

    %Adds additional points at the start and end of interval to allow for
    %linear interpolation across the entire interval, while accounting for
    %the fact that the points might be progressing in the opposite
    %direction
    if (points(1, 1) < points(end, 1))
        points = [xPosTable(1), points(1, 2); points; xPosTable(end), points(end, 2)];
    else
        points = [xPosTable(end), points(1, 2); points; xPosTable(1), points(end, 2)];
    end

    %y_pos_table = interp1(points(:, 1), points(:, 2), pointStart:interval:pointEnd, 'linear');
    yPosTable = interp1(points(:, 1), points(:, 2), xPosTable, 'linear');

    xSize = size(X);

    Xrow = reshape(X,[1, xSize(1)*xSize(2)*xSize(3)] );
    xIndices = floor((Xrow-intStartX)./(intEndX-intStartX)...
                *(size(yPosTable, 2)-2))+1;

    %Performs interpolation between precalculated values
    y0 = yPosTable(xIndices);
    y1 = yPosTable(xIndices+1);
    x0 = xPosTable(xIndices);
    x1 = xPosTable(xIndices+1);

    yPosVals = reshape((y0.*(x1-Xrow)+y1.*(Xrow-x0))./interval, [xSize(1), xSize(2), xSize(3)]);

end
