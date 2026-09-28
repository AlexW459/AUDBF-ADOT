%Name:    Isaac Alexander Nakone, Zehao Liu, Jingya Liu, Kit Nguyen, Alex
%         Welke, Denis Vasilyev.
%Date:    06/07/2024 - 07/07/2024
%Purpose: A function to find the rate of change of aircraft state.
%INPUTS : t == time,
%         s == state of the aircraft.
%           == [x;y;z;U;V;W;P;Q;R;phi;theta;psi];
%         where [x;y;z]         == position,
%               [U,V,W]         == velocity,
%               [P,Q,R]         == angular velocity,
%               [phi,theta,psi] == attitude rates (roll, pitch, yaw).
%         c == the set of constants related to the aircraft configuration.
%           == a struct.
%OUTPUTS: dsdt == the rate of change of state.


function dsdt = SixDOFode(t,s,c, controls)
    dsdt = zeros(12,1); %Because there are 12 state variables
    x = s(1);       %Position x-component
    y = s(2);       %Position y-component
    z = s(3);       %Position z-component
    U = s(4);       %Velocity x-component
    V = s(5);       %Velocity y-component
    W = s(6);       %Velocity z-component
    P = s(7);       %Angular velocity x-component
    Q = s(8);       %Angular velocity y-component
    R = s(9);       %Angular velocity z-component
    phi   = s(10);  %Roll
    theta = s(11);  %Pitch
    psi   = s(12);  %Yaw

    %Determine the thrust, and control surface deflections:
    delta_f = controls(1);
    delta_r = controls(2);
    T       = controls(3);
    
    %Calculate the angle of attack:
    alpha = atan2(W,U); 

    [X,Y,Z] = force_eval(c,U,V,W,phi,theta,psi,alpha,delta_f, delta_r, T);
    [L,M,N] = torque_eval(c,U,V,W,phi,theta,psi,alpha,delta_f, delta_r, T);
    dsdt(1) = 0; %Do we need to correct the velocity?
    dsdt(2) = 0; %Do we need to correct the velocity?
    dsdt(3) = 0; %Do we need to correct the velocity for non-interial rotating reference frame?
    dsdt(4) = X/c.mass - (Q*W-R*V); 
    dsdt(5) = Y/c.mass - (R*U-P*W);
    dsdt(6) = Z/c.mass - (P*V-Q*U);
    dsdt(7:9) = c.inertia_inv*([L;M;N] -[(c.inertia_zz-c.inertia_yy)*Q*R-c.inertia_xz*P*Q;        ...
                                         (c.inertia_xx-c.inertia_zz)*P*R+c.inertia_xz*(P^2-R^2); ...
                                         (c.inertia_yy-c.inertia_xx)*P*Q+c.inertia_xz*Q*R]);
    dsdt(10:12) = [1    sin(phi)*tan(theta)    cos(phi)*tan(theta);
                   0        cos(phi)                -sin(phi);     
                   0    sin(phi)/cos(theta)    cos(phi)/cos(theta)]*[P;Q;R];

end