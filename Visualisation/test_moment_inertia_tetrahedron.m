%Name:
%Date: 23/10/2024
%Purpose: This is a script to test that the moment of inertia calculation
%         is correct based off the numerical example in the paper by F.
%         Tonon. [It is correct]

clear;
clc;
close all;

% Define the vertices of the tetrahedron (example values from the document)
vertices = [
    8.33220, -11.86875, 0.93355;
    0.75523, 5.00000, 16.37072;
    52.61236, 5.00000, -5.38580;
    2.00000, 5.00000, 3.00000
];

% Define the density of the tetrahedron
density = 1.0; % Example value, adjust as needed

% Calculate the inertia tensor
I = tetrahedron_moment_inertia(vertices, density, mean(vertices,1)); 
%Putting in the centrod as the pivot point (3rd argument) gives the correct numerical
%result. In practice, the 3rd argument should be the CG of the aircraft.

% Display the result
disp('Inertia Tensor:');
disp(I);
