function Trimstate = TrimState(Vf, h, gamma)
    % First guess and increments for the Jacobian
    T = 15000;
    dT = 1;
    de = 0 * pi / 180;
    dde = 0.01 * pi / 180;
    Theta = 2 * pi / 180;
    dTheta = dde;

    % Initial forces
    trim = TotalForces(T, de, Theta, Vf, h, gamma);

    Trimstate = [T; de; Theta];

    itercount = 0;
    while max(abs(trim)) > 1e-5
        itercount = itercount + 1;
        % Get value of the function
        trim = TotalForces(T, de, Theta, Vf, h, gamma);

        % Get the Jacobian approximation (3 x 3)
        JT = TotalForces(T + dT, de, Theta, Vf, h, gamma) / dT;
        Jde = TotalForces(T, de + dde, Theta, Vf, h, gamma) / dde;
        JTheta = TotalForces(T, de, Theta + dTheta, Vf, h, gamma) / dTheta;
        Jac = [JT, Jde, JTheta];

        % Get the next iteration
        Trimstate = Trimstate - inv(Jac) * trim;

        T = Trimstate(1);
        de = Trimstate(2);
        Theta = Trimstate(3);
    end

    disp(['Converged after ', num2str(itercount), ' iterations']);
    disp(['For inputs of Vf = ', num2str(Vf, '%1.2f'), ' m/s, h = ', num2str(h / 1e3, '%1.2f'), ' km, gamma = ', num2str(gamma, '%1.2f'), ' deg']);
    disp(['Thrust = ', num2str(T / 1e3, '%1.2f'), ' kN, de = ', num2str(rad2deg(de), '%1.2f'), ' deg, Theta = ', num2str(rad2deg(Theta), '%1.2f'), ' deg']);
end