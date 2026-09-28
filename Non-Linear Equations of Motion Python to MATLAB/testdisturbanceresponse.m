TrimState(120 * 0.5144444, 0, 0);



% Set the inputs
de = -1;
de = deg2rad(de);
h = 0;
Vf = 120 * 0.51444;
gamma = 0;

% Determine the trim state   
trimstate = TrimState(Vf, h, gamma);
alpha = trimstate(3) - gamma;
Ue = Vf * cos(alpha);
We = Vf * sin(alpha);
Qe = 0;
ThetaE = trimstate(3);
Xe = 0;
Ze = h;

% Initial conditions
y0 = [Ue, We, Qe, ThetaE, Xe, Ze, de + trimstate(2)];

% Time vector
t = linspace(0, 100, 1000);

% Solve the differential equations using ode45
[T, Y] = ode45(@(t, y) eqnofmotion800(y, [], trimstate(1)), t, y0);

% Extract the results
U = Y(:, 1);
W = Y(:, 2);
Q = Y(:, 3);
Theta = Y(:, 4);
Alt = Y(:, 6);
alpha = atan2(W, U);
FS = sqrt(U.^2 + W.^2);

% Plot the results
figure;

subplot(3, 2, 1);
plot(T, U);
title('Forward Speed');
xlabel('Time (s)');
ylabel('U (m/s)');

subplot(3, 2, 2);
plot(T, W);
title('Heave Velocity');
xlabel('Time (s)');
ylabel('W (m/s)');

subplot(3, 2, 3);
plot(T, Q);
title('Pitch Rate');
xlabel('Time (s)');
ylabel('Q (deg/s)');

subplot(3, 2, 4);
plot(T, Theta);
title('Pitch Attitude');
xlabel('Time (s)');
ylabel('\theta (deg)');

subplot(3, 2, 5);
plot(Y(:, 5), Alt);
title('Altitude');
xlabel('Horizontal Distance (m)');
ylabel('Alt (m)');

subplot(3, 2, 6);
plot(T, alpha);
title('Angle of Attack');
xlabel('Time (s)');
ylabel('AoA (deg)');

% Store these data for later
tNonlinear = T;
Unonlinear = U;
Wnonlinear = W;
Qnonlinear = Q;
Thetanonlinear = Theta;

TransientSimulation = 0;







%Get required inputs
[Alon, Blon] = numericalDerivatives();

trimstate = TrimState(120 * 0.5144444, 0, 0);

de = trimstate(2);

% Define constants
SIunits = true;

% We will make a B matrix to enable us to use the control system toolbox by
% _exciting the aircraft_ through Elevator input.
% Turn from Zde in 1/radians to 1/degree to put a useful input in.

% Time vector
timeVec = linspace(0, 100, 1000);

% Turn the matrices into a state-space object
LonSS = ss(Alon, Blon, eye(size(Alon, 1)), zeros(size(Blon)));

% Look at the first 100 seconds response to a unit impulse in the only
U = de * ones(size(timeVec));
[~, Y] = lsim(LonSS, U, timeVec);

% Extract the states
u = X(:, 1);
w = X(:, 2);
q = X(:, 3);
theta = X(:, 4);

% Convert q and theta
q = rad2deg(q);
theta = rad2deg(theta);

% Add back on the trim values (assuming Ue, We are defined)

% Plotting
figure;
tiledlayout(2, 2, 'TileSpacing', 'compact');

% Forward Speed
nexttile;
plot(Time, u + Ue, 'b', 'DisplayName', 'Linear');
hold on;
plot(tNonlinear, Unonlinear, 'b--', 'DisplayName', 'Nonlinear');
title('Forward Speed');
xlabel('Time');
ylabel(['u / (' 'm/s' ')']);
legend;

% Heave Velocity
nexttile;
plot(Time, w + We, 'b', 'DisplayName', 'Linear');
hold on;
plot(tNonlinear, Wnonlinear, 'b--', 'DisplayName', 'Nonlinear');
title('Heave Velocity');
xlabel('Time');
ylabel(['w / (' 'm/s' ')']);

% Pitch Rate
nexttile;
plot(Time, q, 'b', 'DisplayName', 'Linear');
hold on;
plot(tNonlinear, Qnonlinear, 'b--', 'DisplayName', 'Nonlinear');
title('Pitch Rate');
xlabel('Time');
ylabel('q / (deg/s)');

% Pitch Attitude
nexttile;
plot(Time, theta, 'b', 'DisplayName', 'Linear');
hold on;
plot(tNonlinear, Thetanonlinear, 'b--', 'DisplayName', 'Nonlinear');
title('Pitch Attitude');
xlabel('Time');
ylabel('θ / deg');

% Adjust labels based on units
if SIunits
    speedlabel = 'm/s';
else
    speedlabel = 'ft/s';
end

