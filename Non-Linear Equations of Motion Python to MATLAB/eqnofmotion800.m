function ydot = eqnofmotion800(y, t, Thrust)
    
% Aircraft Parameters (all in SI)
    m = 7484.4; % mass  = 7484.4kg  etc.
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

    % Get the current values of aircraft states
    dE = y(7); % Elevator input

    U = y(1); % Forward speed
    W = y(2); % Heave velocity
    Q = y(3); % Pitch rate
    Theta = y(4); % Pitch attitude

    % Determine some aerodynamic terms
    Vf = sqrt(U^2 + W^2); % Flight speed
    alpha = atan(W / U); % Angle of attack
    qh = Q * cbar / Vf; % Nondimensional pitch rate

    % Aerodynamic coefficients from Hawker model:
    CL = CL0 + CLa * alpha + CLde * dE;
    CD = CD0 + CDa * alpha + CDa2 * alpha^2;
    CM = CM0 + CMa * alpha + CMde * dE + CMq * qh;

    q_inf = 0.5 * rho * Vf^2 * s;

    % Dimensional aero terms
    lift = q_inf * CL;
    drag = q_inf * CD;
    pm = q_inf * cbar * CM;

    % Simplified nonlinear equations of motion
    ydot = zeros(7, 1);
    ydot(1) = -Q * W + (lift * sin(alpha) - drag * cos(alpha) + Thrust) / m - 9.80665 * sin(Theta); % Udot
    ydot(2) = Q * U + (-lift * cos(alpha) - drag * sin(alpha)) / m + 9.80665 * cos(Theta); % Wdot
    ydot(3) = (pm - Thrust * dZt) / Iyy; % M
    ydot(4) = Q; % ThetaDot
    ydot(5) = U * cos(Theta) - W * sin(Theta); % X_Edot (earth axes)
    ydot(6) = U * sin(Theta) + W * cos(Theta); % Z_Edot (earth axes)
    ydot(7) = 0; % No change to elevator

    return
end