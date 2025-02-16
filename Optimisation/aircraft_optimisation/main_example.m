%Name:    Isaac Nakone, Muhammad Rabay, Alec Vitalievich
%Date:    06/09/2024
%Purpose: Demonstrate how to do optimisation with constraints.

%Simplified problem: 
%Range of a projectile:  Maximise range.


%VARIABLES: -------------------------

%Range:             d,    [m]
%Speed initial:     v,    [m/s] 
%Launch angle:      theta,[deg]
%Initial elevation: y0.   [m] 


%CONSTANT:  -------------------------
%Acceleration due to gravity, g. [m/s^2]


clear;
clc;
close all;


constants.acceleration_gravity = 9.81; %[m/s^2]

my_projectile = projectile(constants);

clear constants;


%Launch angle:
my_projectile.lower_bounds_struct.angle_launch = 0.0;
my_projectile.upper_bounds_struct.angle_launch = 50.0;

%Initial posistion y:
my_projectile.lower_bounds_struct.position_initial_y = 0.0;
my_projectile.upper_bounds_struct.position_initial_y = 0.0;

%Range:
my_projectile.lower_bounds_struct.range = 0.0;
my_projectile.upper_bounds_struct.range = 10000000;

%initial velocity:
my_projectile.lower_bounds_struct.velocity_initial =  0.0;
my_projectile.upper_bounds_struct.velocity_initial = 10000;

%Initial velocity in x-direction:
my_projectile.lower_bounds_struct.velocity_initial_x = 0.0;
my_projectile.upper_bounds_struct.velocity_initial_x = 10000;

%Initial velocity in y-direction:
my_projectile.lower_bounds_struct.velocity_initial_y = 0.0;
my_projectile.upper_bounds_struct.velocity_initial_y = 10000;

%Generate the initial values as the midpoint of the range:
my_projectile.generate_initial_values();

%Optimise the problem:
my_projectile.optimise();

%Display a variable summary:
my_projectile.display_variable_summary();





















