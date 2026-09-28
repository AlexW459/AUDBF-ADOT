function dxdt = dynamics6dof(t,x,c,f)
    %first of all delineate what parts of x mean:
    dxdt = zeros(13,1);

    %dxdt = [ x, y, x, ux, uy, uz, omegax, omegay, omegaz,...
    %         q0, q1, q2, q3];

    vx     = x(4);
    vy     = x(5);
    vz     = x(6);
    omegax = x(7);
    omegay = x(8);
    omegaz = x(9);
    q0     = x(10);
    q1     = x(11);
    q2     = x(12);
    q3     = x(13);

    %organise required vector quantities:
    v       = [vx; vy; vz];
    omega   = [omegax, omegay, omegaz];
    omega_q = [0, omega];
    q       = [q0, q1, q2, q3];
    q = q/norm(q); %Make sure to normalise (to avoid error)
    qc      = [q0, -q1, -q2, -q3];
    uq = mult(mult(q,[0,v']),qc);
    u = uq(2:4)';
    u_infty = norm(u);

    % [phi, theta, psi] = quaternion2euler(q);

    %Find the bearing of the unit vectors of the body frame
    % wrt the ground frame:
    nx = [0, 1, 0, 0];
    ny = [0, 0, 1, 0];
    nz = [0, 0, 0, 1];

    nxp = mult(mult(q,nx),qc);
    nyp = mult(mult(q,ny),qc);
    nzp = mult(mult(q,nz),qc);

    %Find the direction cosines (hence, angles of attack):
    alphax = acos(nxp(2:4)*u/norm(nxp(2:4))/u_infty);
    alphay = acos(nyp(2:4)*u/norm(nyp(2:4))/u_infty);
    alphaz = acos(nzp(2:4)*u/norm(nzp(2:4))/u_infty);

    %Find the magnitude of the lift and drag:
    Ll = 0.5*c.rho*c.Sl*f.CL(alphax, c.left_flap)*u_infty^2;
    Lr = 0.5*c.rho*c.Sr*f.CL(alphax, c.right_flap)*u_infty^2;
    Lt = 0.5*c.rho*c.St*f.CLt(alphax)*u_infty^2;

    %Find the magnitude of the drag forces:
    Dxl = 0.5*c.rho*c.Sl*f.CD(alphax, c.left_flap)*u_infty^2;
    Dxr = 0.5*c.rho*c.Sr*f.CD(alphax, c.right_flap)*u_infty^2;
    Dt  = 0.5*c.rho*c.St*f.CDt(alphax)*u_infty^2;

    %Find the magnitude of the thrust force:
    Tl = c.Tl;
    Tr = c.Tr;
    T  = Tl+Tr;

    %Find the sum of the forces in the X, Y and Z-directions:

    FT  = T*nxp(2:4);
    FDr = -Dxr*nxp(2:4);
    FDl = -Dxr*nxp(2:4);
    FDt = -Dxr*nxp(2:4);

    FLl = -Ll*nzp(2:4);
    FLr = -Lr*nzp(2:4);
    FLt = -Lt*nzp(2:4);




    FX = FT(1)+FDr(1)+FDl(1)+FDt(1)+FLl(1)+FLr(1)+FLt(1);
    FY = FT(2)+FDr(2)+FDl(2)+FDt(2)+FLl(2)+FLr(2)+FLt(2);
    FZ = FT(3)+FDr(3)+FDl(3)+FDt(3)+FLl(3)+FLr(3)+FLt(3);

    %Find the sum of the moments around the Xb,Yb and Zb axes:
    Mx = c.yw*(Ll-Lr) +rand();
    My = -c.xw*(Ll+Lr)+c.xt*Lt+c.zT*T+c.zt*Dt+rand();
    Mz = c.yw*(Dxl-Dxr)+c.yT*(Tl-Tr)+rand();

   

    M = [Mx;My;Mz];

    A = [c.Ix  , 0,  -c.Ixz;
         0     ,c.Iy,   0;
         -c.Ixz, 0,   c.Iz];

    vec = [omegay*omegaz*(c.Iz-c.Iy)-c.Ixz*omegax*omegay;
           omegax*omegaz*(c.Ix-c.Iz)+c.Ixz*(omegax^2-omegaz^2);
           omegax*omegay*(c.Iy-c.Ix)+c.Ixz*omegay*omegaz];

    dxdt(1) = vx;
    dxdt(2) = vy;
    dxdt(3) = vz;
    dxdt(4) = 1/c.m*FX-c.g*sin(theta)-omegay*vz+omegaz*vy;
    dxdt(5) = 1/c.m*FY+c.g*cos(theta)*sin(phi)-omegaz*vx+omegax*vz;
    dxdt(6) = 1/c.m*FZ+c.g*cos(theta)*cos(phi)-omegax*vy+omegay*vx;
    dxdt([7, 8, 9]) = inv(A)*(M - vec);
    dxdt([10,11,12,13]) = 0.5*mult(omega_q, q);


end



