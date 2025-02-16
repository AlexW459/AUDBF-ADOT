%Name:    Alex Welke, Isaac Alexander Nakone
%Date:    09/07/2024 - 09/07/2024
%Purpose: A function to evaluate the torque given planar force [Fx,Fz] and
%         force lever magnitude, l, and force lever angle, b.
%INPUTS : Fx == the force in the body x-direction.
%         Fz == the force in the body z-direction.
%         l  == the force lever arm.
%         b  == the force lever angle.
%OUTPUTS: torque_y == the torque component in y-direction.

function torque_y = planar_torque_equation(Fx,Fz,l,b)
    torque_y = Fx*l*sin(b)-Fz*l*cos(b);
end