function [A, B] = numericalDerivatives(Vf, h, de, gamma)
    if nargin < 1, Vf = 120 * 0.514444; end
    if nargin < 2, h = 0; end
    if nargin < 3, de = deg2rad(1); end
    if nargin < 4, gamma = 0; end

    trimstate = TrimState(Vf, h, gamma);
    T = trimstate(1);
    de = trimstate(2);
    alpha = trimstate(3) - gamma;
    Ue = Vf * cos(alpha);
    We = Vf * sin(alpha);
    Qe = 0;
    ThetaE = trimstate(3);
    Xe = 0;
    Ze = h;

    % Aircraft Parameters
    m = 7484.4; % mass  = 7484.4kg
    Iyy = 84309;
    g = 9.80665;

    % Perturbations
    dUe = 2; %Vf * 0.01
    dWe = 2; %Vf * 0.01
    dQe = 0.1;
    dde = de * 0.01;

    % Derivatives wrt U
    [Xpde, Zpde, Mpde] = perturbationalForces(Ue + dUe, We, Qe, ThetaE, de, h, gamma, T);
    [Xmde, Zmde, Mmde] = perturbationalForces(Ue - dUe, We, Qe, ThetaE, de, h, gamma, T);
    Xu = (Xpde - Xmde) / (2 * dUe) / m;
    Zu = (Zpde - Zmde) / (2 * dUe) / m;
    Mu = (Mpde - Mmde) / (2 * dUe) / Iyy;

    % Derivatives wrt W
    [Xpdw, Zpdw, Mpdw] = perturbationalForces(Ue, We + dWe, Qe, ThetaE, de, h, gamma, T);
    [Xmdw, Zmdw, Mmdw] = perturbationalForces(Ue, We - dWe, Qe, ThetaE, de, h, gamma, T);
    Xw = (Xpdw - Xmdw) / (2 * dWe) / m;
    Zw = (Zpdw - Zmdw) / (2 * dWe) / m;
    Mw = (Mpdw - Mmdw) / (2 * dWe) / Iyy;

    % Derivatives wrt Q
    [Xpdq, Zpdq, Mpdq] = perturbationalForces(Ue, We, Qe + dQe, ThetaE, de, h, gamma, T);
    [Xmdq, Zmdq, Mmdq] = perturbationalForces(Ue, We, Qe - dQe, ThetaE, de, h, gamma, T);
    Xq = (Xpdq - Xmdq) / (2 * dQe) / m;
    Zq = (Zpdq - Zmdq) / (2 * dQe) / m;
    Mq = (Mpdq - Mmdq) / (2 * dQe) / Iyy;

    % Derivatives wrt elevator
    [~, Zpde, Mpde] = perturbationalForces(Ue, We, Qe, ThetaE, de + dde, h, gamma, T);
    [~, Zmde, Mmde] = perturbationalForces(Ue, We, Qe, ThetaE, de - dde, h, gamma, T);
    Zde = (Zpde - Zmde) / (2 * dde) / m;
    Mde = (Mpde - Mmde) / (2 * dde) / Iyy;

    % Put them into an A matrix
    A = [
        Xu, Xw, 0, -g * cos(ThetaE);
        Zu, Zw, Ue, -g * sin(ThetaE);
        Mu, Mw, Mq, 0;
        0, 0, 1, 0
    ];

    disp('At this trim state, the approximation of the A matrix is:');
    disp(A);

    % Control matrix approximation
    B = [
        0; Zde; Mde; 0
    ];

    disp('Control matrix approximation:');
    disp(B);
end