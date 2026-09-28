function EvaluateDisturbance = EvaluateDisturbance()


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
[Time, Y, X] = lsim(LonSS, U, timeVec);

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
ylabel(['u / (' (speedlabel) ')']);
legend;

% Heave Velocity
nexttile;
plot(Time, w + We, 'b', 'DisplayName', 'Linear');
hold on;
plot(tNonlinear, Wnonlinear, 'b--', 'DisplayName', 'Nonlinear');
title('Heave Velocity');
xlabel('Time');
ylabel(['w / (' (speedlabel) ')']);

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



EvaluateDisturbance = 0;
end