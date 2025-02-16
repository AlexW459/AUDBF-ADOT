function F = TotalForces(T, de, Theta, Vf, h, gamma)
    % This gives the total forces for the HS 125 simplified longitudinal aircraft
    % Inputs: Thrust/N, elevator deflection/deg, theta/rad, flightspeed/m/s,
    % flightpath/rad, altitude/m

    % Aircraft Parameters (all in SI)
    m = 7484.4; % mass  = 7484.4kg etc.
    s = 32.8;
    Iyy = 84309;
    cbar = 2.29;
    dZt = -0.378;
    rho = 1.225;
    CD0 = 0.177;
    CDa = 0.232;
    CDa2 = 1.393;
    CL0 = 0.895;
    CLa = 5.01;
    CLde = 0.722;
    CM0 = -0.046;
    CMa = -1.087;
    CMde = -1.88;
    CMq = -7.055;
    g = 9.80665;

    
    % Trim definition
    qh = 0;
    
    % TODO get the density as a function of height
    rho = 1.225;


    % Get the Ue and Ve
    alpha = Theta - gamma;
    CL = CL0 + CLa * alpha + CLde * de;
    CD = CD0 + CDa * alpha + CDa2 * alpha^2;
    CM = CM0 + CMa * alpha + CMde * de + CMq * qh;

    q_infs = 0.5 * rho * Vf^2 * s;

    % Dimensional lift
    lift = q_infs * CL;
    drag = q_infs * CD;
    pm = q_infs * cbar * CM;

    % Determine lift and drag
    F = zeros(3, 1);
    F(1) = T - drag * cos(alpha) + lift * sin(alpha) - m * g * sin(Theta);
    F(2) = -lift * cos(alpha) - drag * sin(alpha) + m * g * cos(Theta);
    F(3) = pm - T * dZt;
end