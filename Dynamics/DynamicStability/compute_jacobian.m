%Alex, Kit Nguyen

%Function:
%        -Inputs: (1) A function f: R^n --> R^n assume x is a column vector f(x).
%                 (2) Given a point x0 in R^n to evaluate J at. 
%        -Outputs: Jacobian matrix J == ( n by n) matrix evaluated at some
%                  point x0.

%The Jacobian matrix below has not been evaluated at x0:

% J(x) = [x(1)     x(2)^2*x(1);
%         x(2)^3   x(1)  ];

%J_{i,j} = \partial f_i(x) / \partial x(j) 


%Example:

% f(x,y) = x^3*sqrt(y);
% (partial f(x,y) / partial x) == 3*x^2*sqrt(y);  
% (partial f(x,y) / partial y) == x^3*0.5/sqrt(y);  

%The Jacobian matrix below has not been evaluated at x0 = [6, -2]:

%x0(1) == 6;
%x0(2) == -2;

% J(x0) =  [6        (-2)^2*6;
%          (-2)^3       6  ];

% J(x0) = [6,   24;
%          -8,  6]

function J_x0 = compute_jacobian(f,x0)

    %Find the dimensions of the matrix 
    %Width is # of input parameters to f
    matrix_width = size(x0, 1);
    %Height is number of output variables from f
    matrix_height = size(f(x0), 1);

    J_x0 = zeros([matrix_height, matrix_width]);
    
    h = 10^(-5);

    for r = 1:matrix_height
        for c = 1:matrix_width
            x0_h = zeros(1, matrix_width);
            x0_h(c) = h;
            x0_upper = x0 + x0_h;
            x0_lower = x0 - x0_h;
            
            f_upper = f(x0_upper);
            f_lower = f(x0_lower);

            J_x0(r, c) = (f_upper(r)-f_lower(r))/(2*h);
        end
    end
end
