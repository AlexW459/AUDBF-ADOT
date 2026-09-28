#include "edgeSDF.h"

using namespace std;

void edgeSDF(glm::dvec3 pts[6], int dir, const vector<double>& xVals, const vector<double>& yVals, 
    const vector<double>& zVals, vector<double>& SDF){

    int xSize = xVals.size();
    int ySize = yVals.size();
    int zSize = zVals.size();

    double xMax = -INF, xMin = INF;
    double yMax = -INF, yMin = INF;
    double zMax = -INF, zMin = INF;

    // Gets axis-aligned bounding box of face extrusion
    for (int i = 0; i < 6; i++) {
        xMax = max(xMax, pts[i][0]);
        xMin = min(xMin, pts[i][0]);
        yMax = max(yMax, pts[i][1]);
        yMin = min(yMin, pts[i][1]);
        zMax = max(zMax, pts[i][2]);
        zMin = min(zMin, pts[i][2]);
    }

    //for(int i = 0; i < 6; i++){
    //    cout << pts[i][0] << ", " << pts[i][1] << ", " << pts[i][2] << " | ";
    //}cout << endl;

    int xMaxIdx = floor(pointPosToIndex(xVals[0], xVals[xSize-1], xSize, xMax));
    int yMaxIdx = floor(pointPosToIndex(yVals[0], yVals[ySize-1], ySize, yMax));
    int zMaxIdx = floor(pointPosToIndex(zVals[0], zVals[zSize-1], zSize, zMax));

    int xMinIdx = ceil(pointPosToIndex(xVals[0], xVals[xSize-1], xSize, xMin));
    int yMinIdx = ceil(pointPosToIndex(yVals[0], yVals[ySize-1], ySize, yMin));
    int zMinIdx = ceil(pointPosToIndex(zVals[0], zVals[zSize-1], zSize, zMin));

    /*int xMaxIdx = ceil(pointPosToIndex(xVals[0], xVals[xSize-1], xSize, xMax));
    int yMaxIdx = ceil(pointPosToIndex(yVals[0], yVals[ySize-1], ySize, yMax));
    int zMaxIdx = ceil(pointPosToIndex(zVals[0], zVals[zSize-1], zSize, zMax));

    int xMinIdx = floor(pointPosToIndex(xVals[0], xVals[xSize-1], xSize, xMin));
    int yMinIdx = floor(pointPosToIndex(yVals[0], yVals[ySize-1], ySize, yMin));
    int zMinIdx = floor(pointPosToIndex(zVals[0], zVals[zSize-1], zSize, zMin));*/

    glm::dvec3 planeEqs[5];
    double planeConstants[5];

    planeEqs[0] = glm::cross(pts[4]-pts[0],pts[2]-pts[0]);
    planeConstants[0] = -glm::dot(planeEqs[0], pts[0]);
    planeEqs[1] = glm::cross(pts[3]-pts[1],pts[5]-pts[1]);
    planeConstants[1] = -glm::dot(planeEqs[1], pts[1]);
    planeEqs[2] = glm::cross(pts[2]-pts[0],pts[3]-pts[0]);
    planeConstants[2] = -glm::dot(planeEqs[2], pts[0]);
    planeEqs[3] = glm::cross(pts[4]-pts[2],pts[5]-pts[2]);
    planeConstants[3] = -glm::dot(planeEqs[3], pts[2]);
    planeEqs[4] = glm::cross(pts[1]-pts[0],pts[5]-pts[0]);
    planeConstants[4] = -glm::dot(planeEqs[4], pts[0]);


    for(int x = xMinIdx; x <= xMaxIdx; x++){
        for(int y = yMinIdx; y <= yMaxIdx; y++){
            for(int z = zMinIdx; z <= zMaxIdx; z++){
                //Checks each point inside box
                bool isIn = true;
                
                glm::dvec3 pCoords(xVals[x], yVals[y], zVals[z]);
                for (int i = 0; i < 5; i++) {
                    if (glm::dot(planeEqs[i], pCoords) + planeConstants[i] > 1e-12) {
                        isIn = false;
                    }
                }
                if(isIn){
                    //Gets distance to edge
                    int SDFindex = meshIndexTo1DIndex(x, y, z, xVals.size(), yVals.size());
                    glm::dvec3 ab = glm::normalize(pts[1]-pts[0]);
                    glm::dvec3 av = pCoords - pts[0];
                    double t = glm::dot(ab, av);
                    glm::dvec3 h = pts[0] + t*ab;
                    double dist = glm::length(pCoords - h);
                    
                    if(dist < fabs(SDF[SDFindex])){
                        SDF[SDFindex] = (double)dir*dist;
                    }
                }
            }
        }
    }
}
