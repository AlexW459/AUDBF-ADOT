
[X, Y] = meshgrid(-2:0.02:2, -2:0.02:2);

points = [];

for i=1:5
    points(i, :) = [cosd(i/10*360), sind(i/10*360)]; 
end

%points = [points; 0, 0];

plot([points(:, 1); points(1, 1)], [points(:, 2); points(1, 2)], "Color", 'r');


x1 = points(1:end, 1);
y1 = points(1:end, 2);
x2 = [points(2:end, 1); points(1, 1)];
y2 = [points(2:end, 2); points(1, 2)];


%Find parameters of implicit equation of side of polygon
a1 = y2-y1;
b1 = x1 - x2;
c1 = x2.*y1 - x1.*y2;

l2 = (x1-x2).^2 + (y1-y2).^2;
v = [x1, y1];
w = [x2, y2];
diff = w-v;
        
distances = X;

for i = 1:size(X, 1)
    for j = 1:size(X, 2)
        %Find distance from point to nearest line segment
        p = [X(i, j), Y(i, j)];

        t = max([zeros(size(v, 1), 1), min([ones(size(v, 1), 1), dot(p-v, w-v, 2) ./ l2], [], 2)], [], 2);

        proj = v + [t .*diff(:, 1), t.*diff(:, 2)];
        dist = p-proj;
        distances(i, j) = min(sqrt(dist(:, 1).^2 + dist(:, 2).^2));

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
        %of the polygon. Ceil is used so that if either point lies on the
        %polygon and the value of d1 or d2 is zero, the value of the
        %heaviside function gets rounded up to 1
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

        distances(i, j) = distances(i, j).*distSign;

    end
end


hold on;
surf(X, Y, distances);
