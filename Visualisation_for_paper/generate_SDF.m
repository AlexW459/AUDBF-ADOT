%Name:        Alex Welke
%Date:        5/07/2025
%Description: This function generates a signed distance field based on a
%set of points outlining a polygon

function SDF = generate_SDF(vertices, X, Y)

    x1 = vertices(1:end, 1);
    y1 = vertices(1:end, 2);
    x2 = [vertices(2:end, 1); vertices(1, 1)];
    y2 = [vertices(2:end, 2); vertices(1, 2)];

    a1 = y2-y1;
    b1 = x1 - x2;
    c1 = x2.*y1 - x1.*y2;



    %Precalculates values for relevant range
    intStartX = min(X, [],"all");
    intEndX = max(X, [], "all");
    intStartY = min(Y, [],"all");
    intEndY = max(Y, [], "all");
    intSize = [intEndX - intStartX, intEndY - intStartY];
    xSize = size(X);

    valSize = round(20*xSize);
    xVals = linspace(intStartX, intEndX, valSize(1));
    xInterval = xVals(2) - xVals(1);
    yVals = linspace(intStartY, intEndY, valSize(2));
    yInterval = yVals(2) - yVals(1);

    SDFtable = zeros(valSize(1), valSize(2));

    %Choose [100, 0] as a point that will definitely be outside the
    %polygon. This point and the current coordinate form a line
    pOut = [100.245, 150.245];


    for i = 1:valSize(1)
        for j = 1:valSize(2)
            p = [xVals(i), yVals(j)];
            %Find if point is inside or outside polygon

   

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
            int1 = d1.*d2 < 0;

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
            int2 = d1.*d2 < 0;


            %The sum of the two int arrays multiplied together is equal to the
            %total number of intersections between the outgoing line and the
            %side of the polygon. If this sum is odd then the point must be
            %inside the polygon, and so the SDF is made negative
            distSign = 1-2*mod(sum(int1.*int2, 1), 2);

            SDFtable(i, j) = distSign;%*halfInterval

        end
    end

    

    toc

    %Find location of X and Y values in table of values
    Xrow = reshape(X, [1, xSize(1)*xSize(2)*xSize(3)]);
    xIndices = floor((Xrow-intStartX)./intSize(1)...
                *(valSize(1)-2))+1;
    Yrow = reshape(Y, [1, xSize(1)*xSize(2)*xSize(3)]);
    yIndices = floor((Yrow-intStartY)./intSize(2)...
                *(valSize(2)-2))+1;


    SDF = reshape(SDFtable(sub2ind(size(SDFtable), xIndices, yIndices)), [xSize(1), xSize(2), xSize(3)]);

    %2D interpolation
    x0 = xVals(xIndices);
    x1 = xVals(xIndices+1);
    y0 = yVals(yIndices);
    y1 = yVals(yIndices+1);

    Q00 = SDFtable(sub2ind(size(SDFtable), xIndices, yIndices));
    Q10 = SDFtable(sub2ind(size(SDFtable), xIndices+1, yIndices));
    Q01 = SDFtable(sub2ind(size(SDFtable), xIndices, yIndices+1));
    Q11 = SDFtable(sub2ind(size(SDFtable), xIndices+1, yIndices+1));

    diff1 = (x1-Xrow)./xInterval;
    diff2 = (Xrow-x0)./xInterval;

    SDFy0 = diff1.*Q00 + diff2.*Q10;
    SDFy1 = diff1.*Q01 + diff2.*Q11;

    SDFlist = (y1-Yrow)./yInterval.*SDFy0 + (Yrow-y0)./yInterval.*SDFy1;

    %SDF = reshape(SDFlist, [xSize(1), xSize(2), xSize(3)]);

end