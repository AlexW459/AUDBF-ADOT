#include "fastsweep.h"

// Algorithm: https://epubs.siam.org/doi/10.1137/10080909X
// Code: https://mathworks.com/matlabcentral/fileexchange/105620-fast-sweeping-method-in-2d-and-3d

// Modified to only calculate within narrow band

using namespace std;

void fastSweep(vector<double>& field, int xSize, int ySize, int zSize, double h, int nSweeps){


    //Gets all grid points at positive/negative interfae
    int totalFieldSize = xSize*ySize*zSize;
    // 0: inside band, 1: on interface, 2: outside band
    vector<char> cellType(totalFieldSize, 0);
    int xySize = xSize*ySize;
    for(int i = xySize; i < totalFieldSize-xySize; i++){
        double cur = field[i];
        if(field[i] == INF || field[i] == -INF){
            cellType[i] = 2;
        }
        else if(field[i+1]*cur < 0.0 || field[i-1]*cur < 0.0 ||
            field[i+xSize]*cur < 0.0 || field[i-xSize]*cur < 0.0 ||
            field[i+xySize]*cur < 0.0 || field[i-xySize]*cur < 0.0) {
            cellType[i] = 1;
        }
    }

    // Start and end points for sweeps in each direction
    const glm::ivec2 xLoop[8] = {{1, xSize-1}, {1, xSize-1}, {xSize-1, 1}, {xSize-1, 1}, {1, xSize}, {1, xSize}, {xSize, 1}, {xSize, 1}};
    const int xSign[8] = {1, 1, -1, -1, 1, 1, -1, -1};
    const glm::ivec2 yLoop[8] = {{1, ySize-1}, {ySize-1, 1}, {1, ySize-1}, {ySize-1, 1}, {1, ySize}, {ySize, 1}, {1, ySize}, {ySize, 1}};
    const int ySign[8] = {1, -1, 1, -1, 1, -1, 1, -1};
    const glm::ivec2 zLoop[8] = {{1, zSize-1}, {1, zSize-1}, {1, zSize-1}, {1, zSize-1}, {zSize, 1}, {zSize, 1}, {zSize, 1}, {zSize, 1}};
    const int zSign[8] = {1, 1, 1, 1, -1, -1, -1, -1};

    // Each sweep involves a sweep in each of 8 directions
    for(int i = 0; i < nSweeps*8; i++){
        // Whether the field was changed at all on the last iteration
        //bool changed = false;


        // Gets direction of sweep
        int dir = i % 8;

        for(int x = xLoop[dir][0]; x < xLoop[dir][1]; x += xSign[dir]){
            for(int y = yLoop[dir][0]; y < yLoop[dir][1]; y += ySign[dir]){
                for(int z = zLoop[dir][0]; z < zLoop[dir][1]; z += zSign[dir]){
                    int fieldIndex = meshIndexTo1DIndex(x, y, z, xSize, ySize);

                    if(cellType[fieldIndex] == 0){
                        // Apply sweep
                    }
                    
                }
            }
        }




    }

}