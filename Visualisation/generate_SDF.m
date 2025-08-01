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
    a1 = y2-y1;
    b1 = x1 - x2;
    c1 = x2.*y1 - x1.*y2;
    
    l2 = (x1-x2).^2 + (y1-y2).^2;
    v = [x1, y1];
    w = [x2, y2];
    diff = w-v;


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
            %Find if point is inside or outside polygon

            %Choose [100, 0] as a point that will definitely be outside the
            %polygon. This point and the current coordinate form a line
            pOut = [100.245, 150.245];

            %Find which side of the line segment the outside point is on
            d1 = a1.*p(1) + b1.*p(2) + c1;
            %Find which side of the line segment the inside point is on
            d2 = a1.*pOut(1) + b1.*pOut(2) + c1;

            %If d1 and d2 have the same sign, their product is positive and so
            %the heaviside function is equal to 0. This indicates that there is
            %no intersection between the line going to the outside and the side
            %of the polygon. Floor is used so that if either point lies on the
            %polygon and the value of d1 or d2 is zero, the value of the
            %heaviside function gets rounded down to zero
            int1 = floor(heaviside(-1*d1.*d2));


            %Find parameters of implicit equation of line going to outside of
            %polygon
            a2 = pOut(2)-p(2);
            b2 = p(1) - pOut(1);
            c2 = pOut(1)*p(2) - p(1)*pOut(2);

            %Finds whether the two vertices at either end of the side of the
            %polygon are on either side of the line going to the outside, as
            %this indicates a possible intersection between the lines
            d1 = a2.*x1 + b2.*y1 + c2;
            d2 = a2.*x2 + b2.*y2 + c2;


            %If d1 and d2 have the same sign, their product is positive and so
            %the heaviside function is equal to 0. This indicates that there is
            %no intersection between the line going to the outside and the side
            %of the polygon
            int2 = floor(heaviside(-1*d1.*d2));


            %The sum of the two int arrays multiplied together is equal to the
            %total number of intersections between the outgoing line and the
            %side of the polygon. If this sum is odd then the point must be
            %outside the polygon
            distSign = 1-2*mod(sum(int1.*int2, 1), 2);

            %Find distance from point to nearest line segment

            t = max([zeros(size(v, 1), 1), min([ones(size(v, 1), 1), dot(p-v, w-v, 2) ./ l2], [], 2)], [], 2);

            proj = v + [t .*diff(:, 1), t.*diff(:, 2)];
            dist = p-proj;

            SDFtable(i, j) = min(sqrt(dist(:, 1).^2 + dist(:, 2).^2))*distSign;

        end
    end

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