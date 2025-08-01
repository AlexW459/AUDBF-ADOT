%Name:        Alex Welke
%Date:        5/07/2025
%Description: This function generates a signed distance field based on a
%set of points outlining a polygon

function SDF = generate_SDF(vertices, X, Y, interval)

    x1 = vertices(1:end, 1);
    y1 = vertices(1:end, 2);
    x2 = [vertices(2:end, 1); vertices(1, 1)];
    y2 = [vertices(2:end, 2); vertices(1, 2)];

    %Find parameters of implicit equation of side of polygon
    l2 = (x1-x2).^2 + (y1-y2).^2;
    v = [x1, y1];
    w = [x2, y2];
    diff = w-v;

    norms = [-1.*diff(:, 2), diff(:, 1)];


    %Precalculates values for relevant range
    intStartX = min(X, [],"all");
    intEndX = max(X, [], "all");
    intStartY = min(Y, [],"all");
    intEndY = max(Y, [], "all");
    xSize = size(X);

    xVals = intStartX:interval:intEndX;
    yVals = intStartY:interval:intEndY;


    SDFtable = zeros(size(xVals, 2), size(yVals, 2));

    
    for i = 1:size(xVals, 2)
        for j = 1:size(yVals, 2)
            p = [xVals(i), yVals(j)];
            %Find distance from point to nearest line segment


            dot_prods = dot(p-v, diff, 2);
            t = max([zeros(size(v, 1), 1), min([ones(size(v, 1), 1), dot_prods ./ l2], [], 2)], [], 2);

            proj = v + [t .*diff(:, 1), t.*diff(:, 2)];
            dists = p-proj;

            [dist, elem] =  min(sqrt(dists(:, 1).^2 + dists(:, 2).^2));

            SDFtable(i, j) = dist*sign(dot(norms(elem, :), dists(elem, :)));

        end
    end

    toc

    %Find location of X and Y vales in table of values
    Xrow = reshape(X, [1, xSize(1)*xSize(2)*xSize(3)]);
    xIndices = floor((Xrow-intStartX)./(intEndX-intStartX)...
                *(size(xVals, 2)-2))+1;
    Yrow = reshape(Y, [1, xSize(1)*xSize(2)*xSize(3)]);
    yIndices = floor((Yrow-intStartY)./(intEndY-intStartY)...
                *(size(yVals, 2)-2))+1;

    %Perform 2d interpolation
    x0 = xVals(xIndices);
    x1 = xVals(xIndices+1);
    y0 = yVals(yIndices);
    y1 = yVals(yIndices+1);

    Q00 = SDFtable(sub2ind(size(SDFtable), xIndices, yIndices));
    Q10 = SDFtable(sub2ind(size(SDFtable), xIndices+1, yIndices));
    Q01 = SDFtable(sub2ind(size(SDFtable), xIndices, yIndices+1));
    Q11 = SDFtable(sub2ind(size(SDFtable), xIndices+1, yIndices+1));

    diff1 = (x1-Xrow)./interval;
    diff2 = (Xrow-x0)./interval;

    SDFy0 = diff1.*Q00 + diff2.*Q10;
    SDFy1 = diff1.*Q01 + diff2.*Q11;

    SDFlist = (y1-Yrow)./interval.*SDFy0 + (Yrow-y0)./interval.*SDFy1;

    SDF = reshape(SDFlist, [xSize(1), xSize(2), xSize(3)]);

end