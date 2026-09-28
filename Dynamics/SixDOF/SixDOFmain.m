%Name:    Isaac Alexander Nakone, Zehao Liu, Jingya Liu, Kit Nguyen, Alex
%         Welke, Denis Vasilyev.
%Date:    06/07/2024 - 07/07/2024
%Purpose: Simulate the 6DOF EOMs for aircraft.


clear;
clc;
close all;

%Constant list:
c.inertia_xx = 20;
c.inertia_yy = 20;
c.inertia_zz = 20;
c.inertia_xz = 10;
c.inertia =[c.inertia_xx    0   -c.inertia_xz;
             0      c.inertia_yy    0;
           -c.inertia_xz   0    c.inertia_zz];
c.inertia_inv = inv(c.inertia);
c.mass = 10;

