%Name:    Isaac Alexander Nakone, Zehao Liu, Jingya Liu, Kit Nguyen, Alex
%         Welke, Denis Vasilyev.
%Date:    06/07/2024 - 08/07/2024
%Purpose: Given a constant velocity [U0,0,0], and constant angular velocity 
%         [P,Q,R] = [0,0,0] (attitude rates) (phi = psi = 0, theta0 =/= 0)
%         find trim thrust T, trim pitch theta0, trim delta_r and trim
%         delta_f.


%Define theta0 == 0.
%Define alpha == 0.


%Constant list:
% c.inertia_xx = 20;
% c.inertia_yy = 20;
% c.inertia_zz = 20;
% c.inertia_xz = 10;
% c.inertia =[c.inertia_xx    0   -c.inertia_xz;
%              0      c.inertia_yy    0;
%            -c.inertia_xz   0    c.inertia_zz];
% c.inertia_inv = inv(c.inertia);
% c.mass = 10;

clear;
clc;
close all;

%Constant list:

c.mass = 8;
c.gravity = 9.8;



c.inertia_yy = 20;

c.inertia_xx = 20;
c.inertia_zz = 20;
c.inertia_xz = 10;
c.inertia =[c.inertia_xx    0   -c.inertia_xz;
             0      c.inertia_yy    0;
           -c.inertia_xz   0    c.inertia_zz];
c.inertia_inv = inv(c.inertia);



%Used
%(https://www.bigfoil.com/F/ed677818-5a94-4df6-8006-2eebed47df40_infoF1.php)
% To get the airfoil data for the GOE567, and NACA0012
c.lift_coeff_alpha_wing = 5.73;
c.lift_coeff_delta_wing = 5.1;
c.lift_coeff_wing_0     = 0.75;

c.lift_coeff_alpha_tail = 6.0;
c.lift_coeff_delta_tail = 2.4;
c.lift_coeff_tail_0     = 0.0;

c.drag_coeff_alpha_wing = 0.115;
c.drag_coeff_delta_wing = 0.076;
c.drag_coeff_wing_0     = 0.01;

c.drag_coeff_alpha_tail = 0.115;
c.drag_coeff_delta_tail = 0.13;
c.drag_coeff_tail_0     = 0.008;

c.rho = 1.2;
c.area_wing_planform = 0.269;
c.area_tail_planform = 0.08;

c.lever_arm_tail   = 0.7801;
c.lever_arm_wing   = 0.122;
c.lever_arm_thrust = 0.255;

c.lever_angle_tail   = 192*(pi/180);
c.lever_angle_wing   = 224.5*(pi/180);
c.lever_angle_thrust = -11*(pi/180);

U0 = linspace(20,200,100);
theta = 0*pi/180;
delta_f = zeros(1,size(U0,2));
delta_r = zeros(1,size(U0,2));
thrust = zeros(1,size(U0,2));


for velocity_index = 1:size(U0,2)
    trim = fsolve(@(trim_var)trim_function(c,U0(velocity_index),0,0,0,theta,0,0,trim_var),...
                   [0,0,200]);
    delta_f(velocity_index) = 180/pi*trim(1);
    delta_r(velocity_index) = 180/pi*trim(2);
    thrust(velocity_index) = trim(3);
end


index = 50;
tspan = linspace(0,200,1000);
s0 = [0,0,0,U0(index),0,0,0,0,0,0,10*pi/180,0];
controls = [pi/180*delta_f(index),pi/180*delta_r(index),thrust(index)];

[t,s] = ode45(@(t,s)SixDOFode(t,s,c, controls), tspan, s0 );

plot(t,180/pi*s(:,11));
title(['Longitudinal dynamic stability, $U_0 = $ ', num2str(U0(index)), ' m/s'],'Interpreter','latex');
xlabel('time (s)','Interpreter','latex');
ylabel('$\theta$ $(^{\circ})$','Interpreter','latex');




% subplot(1,2,1)
% 
% plot(U0,delta_r,'r',U0,delta_f,'b');
% title('Controls at trim flight','interpreter','latex');
% legend('$\delta_{r}$','$\delta_{f}$','interpreter','latex');
% xlabel('$x_B$-component of velocity, $U_0$ (m/s)','interpreter','latex');
% ylabel('Angular deflection, $(^{\circ})$','interpreter','latex');
% 
% subplot(1,2,2)
% plot(U0,thrust,'g');
% title('Controls at trim flight','interpreter','latex');
% xlabel('$x_B$-component of velocity, $U_0$ (m/s)','interpreter','latex');
% ylabel('Thrust $(N)$','interpreter','latex');




function trim_forces = trim_function(c,U,V,W,phi,theta,psi,alpha,trim_var)

    delta_f = trim_var(1);
    delta_r = trim_var(2);
    T       = trim_var(3);

    [X,Y,Z] = force_eval(c,U,V,W,phi,theta,psi,alpha,delta_f, delta_r, T);
    [L,M,N] = torque_eval(c,U,V,W,phi,theta,psi,alpha,delta_f, delta_r, T);

    trim_forces = [X,Z,M];

end






