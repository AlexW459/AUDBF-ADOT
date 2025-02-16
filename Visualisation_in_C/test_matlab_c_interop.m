%Name:        ADOT team
%Date:        09/08/2024
%Description: This code uses matlab coder
%             to call a C script.

clear;
clc;
close all;

load("primitives.mat");

facets2nodes = primitives_data_base.icosahedron.facets2nodes-1; %converting to C indexing.
nodes2coords = primitives_data_base.icosahedron.nodes2coords;
facets2nodes = facets2nodes';
nodes2coords = nodes2coords';
facets2nodes = facets2nodes(:);
nodes2coords = nodes2coords(:);

mkdir("icosahedron");
save("icosahedron/nodes2coords.txt","nodes2coords","-ascii");

fileID = fopen('icosahedron/facets2nodes.txt','w');
for i = 1:size(facets2nodes,1)
    fprintf(fileID,'  %u\n',facets2nodes(i));
end
fclose(fileID);

% mex -setup c;
% mex( 'src/main0.c', 'src/glad.c','-Iinclude','-Llib','-lglfw3dll');
% 
% movefile main0.mexw64 src

