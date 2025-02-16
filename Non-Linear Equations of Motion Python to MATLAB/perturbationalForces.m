function [Fx, Fz, My] = perturbationalForces(U, W, Q, Theta, de, h, gamma, T)
    % Aircraft Parameters (all in SI)
    m = 7484.4; % mass  = 7484.4kg
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

    qh = 0;  % Trim definition
    
    % TODO get the density as a function of height
    rho = 1.225;
    
    % Get the Ue and We and add the perturbational values
    alpha = atan2(W, U);
    Vf = sqrt(U^2 + W^2);
    Qhat = Q * cbar / Vf;

    % Recalculate alpha    
    CL = CL0 + CLa * alpha + CLde * de;
    CD = CD0 + CDa * alpha + CDa2 * alpha^2;
    CM = CM0 + CMa * alpha + CMde * de + CMq * Qhat;

    q_infs = 0.5 * rho * Vf^2 * s;

    % Dimensional lift
    lift = q_infs * CL;
    drag = q_infs * CD;
    pm = q_infs * cbar * CM;

    % Determine lift and drag
    Fx = T - drag * cos(alpha) + lift * sin(alpha) - m * g * sin(Theta);
    Fz = -lift * cos(alpha) - drag * sin(alpha) + m * g * cos(Theta);
    My = pm - T * dZt;
end
