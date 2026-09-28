%Name: Alex
%Date: 16/09/2024
%Description: Creates an object file of a mesh given a list of node
%coordinates and a list of the nodes in each triangle


function writeMeshtoObj(nodeCoords, triangleNodes, fileName)
    fileNameFull = [convertStringsToChars(fileName), '.obj'];
    fileID = fopen(fileNameFull,'w');

    for i = 1:size(nodeCoords, 1)
        fprintf(fileID,'v %0.6f %0.6f %0.6f\n', nodeCoords(i, 1), nodeCoords(i, 2), nodeCoords(i, 3));
    end

    for i = 1:size(triangleNodes, 1)
        fprintf(fileID,'f %d %d %d\n', triangleNodes(i, 1), triangleNodes(i, 2), triangleNodes(i, 3));
    end

    fclose(fileID);

end