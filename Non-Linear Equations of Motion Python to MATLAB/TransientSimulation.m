function TransientSimulation = TransientSimulation()

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

end