%Name:    Isaac Alexander Nakone, Zehao Liu, Jingya Liu, Kit Nguyen, Alex
%         Welke, Denis Vasilyev.
%Date:    06/07/2024 - 08/07/2024
%Purpose: A function to evaluate the torques on the aircraft
%         based on the current configuration.


function [L,M,N] = torque_eval(c,U,V,W,phi,theta,psi,alpha,delta_f, delta_r, T)
    speed = norm([U,V,W],2); %The speed is the magnitude of velocity

    %V-tail (ruddervator) == horizontal stabiliser (elevator) and a vertical stabiliser (rudder).
    % Controls:
    % delta_ruddervator == delta_r
    % delta_flaperon == delta_f
    % Thrust.

    lift_coeff_wing = c.lift_coeff_alpha_wing*alpha+...
                      c.lift_coeff_delta_wing*delta_f+c.lift_coeff_wing_0; 
    lift_coeff_tail = c.lift_coeff_alpha_tail*alpha+...
                      c.lift_coeff_delta_tail*delta_r+c.lift_coeff_tail_0;
    drag_coeff_wing = c.drag_coeff_alpha_wing*alpha+...
                      c.drag_coeff_delta_wing*delta_f+c.drag_coeff_wing_0;
    drag_coeff_tail = c.drag_coeff_alpha_tail*alpha+...
                      c.drag_coeff_delta_tail*delta_r+c.drag_coeff_tail_0;

    Lw = 0.5*lift_coeff_wing*c.rho*c.area_wing_planform*speed^2; %lift force on the wing  
    Lt = 0.5*lift_coeff_tail*c.rho*c.area_tail_planform*speed^2; %lift force on the tail  
    Dw = 0.5*drag_coeff_wing*c.rho*c.area_wing_planform*speed^2; %Drag force on the wing 
    Dt = 0.5*drag_coeff_tail*c.rho*c.area_tail_planform*speed^2; %Drag force on the tail

    % D = Dw+Dt;
    % L = Lw+Lt;

    %alpha_prime == angle between velocity and ground (xB').
    %alpha       == angle between velocity and heading (xB).
    %alpha_prime + alpha = theta.

    %alpha_prime = theta-alpha; %Don't need this as of 09/07/2024
    %(Alex and Isaac).

    

    L = 0;
    M = planar_torque_equation(T,0,c.lever_arm_thrust,c.lever_angle_thrust)+...
        planar_torque_equation(Lw*sin(alpha),-Lw*cos(alpha),c.lever_arm_wing,c.lever_angle_wing)+...
        planar_torque_equation(Lt*sin(alpha),-Lt*cos(alpha),c.lever_arm_tail,c.lever_angle_tail)+...
        planar_torque_equation(-Dw*cos(alpha),-Dw*sin(alpha),c.lever_arm_wing,c.lever_angle_wing)+...
        planar_torque_equation(-Dt*cos(alpha),-Dt*sin(alpha),c.lever_arm_tail,c.lever_angle_tail);

    % changed from "-Lt*c.lever_lift_tail-Dt*c.lever_drag_tail-Dw*c.lever_drag_wing+...
    %   Lw*c.lever_lift_wing+T*c.lever_thrust" on 09/07/2024 by Alex and Isaac
    N = 0;
end