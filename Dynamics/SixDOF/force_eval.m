%Name:    Isaac Alexander Nakone, Zehao Liu, Jingya Liu, Kit Nguyen, Alex
%         Welke, Denis Vasilyev.
%Date:    06/07/2024 - 09/07/2024
%Purpose: A function to evaluate the forces on the aircraft
%         based on the current configuration.



function [X,Y,Z] = force_eval(c,U,V,W,phi,theta,psi,alpha,delta_f, delta_r, T)
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
    drag_coeff_wing = c.drag_coeff_alpha_wing*abs(alpha)+...
                      c.drag_coeff_delta_wing*abs(delta_f)+c.drag_coeff_wing_0; % Alex and Isaac added abs() on 10/07/2024
    drag_coeff_tail = c.drag_coeff_alpha_tail*abs(alpha)+...
                      c.drag_coeff_delta_tail*abs(delta_r)+c.drag_coeff_tail_0; % Alex and Isaac added abs() on 10/07/2024

    Lw = 0.5*lift_coeff_wing*c.rho*c.area_wing_planform*speed^2; %lift force on the wing  
    Lt = 0.5*lift_coeff_tail*c.rho*c.area_tail_planform*speed^2; %lift force on the tail  
    Dw = 0.5*drag_coeff_wing*c.rho*c.area_wing_planform*speed^2; %Drag force on the wing 
    Dt = 0.5*drag_coeff_tail*c.rho*c.area_tail_planform*speed^2; %Drag force on the tail

    D = Dw+Dt;
    L = Lw+Lt;

    %alpha_prime == angle between velocity and ground (xB').
    %alpha       == angle between velocity and heading (xB).
    %alpha_prime + alpha = theta.

    % alpha_prime = theta-alpha; %Commented this out on  09/07/2024.
    % Because we don't need alpha_prime anymore.
    
    %We realised (on 09/07/2024) that [X,Y,Z] refer to the forces in the
    %body frame B, not the frame B'.
    X = T+(Lw+Lt)*sin(alpha)-(Dw+Dt)*cos(alpha)-c.mass*c.gravity*sin(theta);  
    % changed from "-D*cos(alpha_prime)-L*sin(alpha_prime)+T*cos(theta)" on 09/07/2024 by Alex and Isaac
    Y = c.mass*c.gravity*sin(phi)*cos(theta);
    % changed from "0" on 09/07/2024 by Alex and Isaac
    Z = -(Lw+Lt)*cos(alpha)-(Dw+Dt)*sin(alpha)+c.mass*c.gravity*cos(theta);
    % changed from "c.mass*c.gravity-D*sin(alpha_prime)-L*cos(alpha_prime)-T*sin(theta)" on 09/07/2024 by Alex and Isaac
end