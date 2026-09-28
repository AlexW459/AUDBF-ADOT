#include "vertexSDF.h"

using namespace std;

void vertexSDF(glm::dvec3 vert, double radius, glm::dvec3 axis, double angle, int dir, const vector<double>& xVals, 
    const vector<double>& yVals, const vector<double>& zVals, vector<double>& SDF){

    int xSize = xVals.size();
    int ySize = yVals.size();
    int zSize = zVals.size();

    double xMax = -INF, xMin = INF;
    double yMax = -INF, yMin = INF;
    double zMax = -INF, zMin = INF;

    // Gets axis-aligned bounding box of face extrusion
    xMax = max(xMax, vert[0] + radius);
    xMin = min(xMin, vert[0] - radius);
    yMax = max(yMax, vert[1] + radius);
    yMin = min(yMin, vert[1] - radius);
    zMax = max(zMax, vert[2] + radius);
    zMin = min(zMin, vert[2] - radius);


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


    for(int x = xMinIdx; x <= xMaxIdx; x++){
        for(int y = yMinIdx; y <= yMaxIdx; y++){
            for(int z = zMinIdx; z <= zMaxIdx; z++){
                glm::dvec3 d = glm::dvec3(xVals[x], yVals[y], zVals[z]) - vert;
                double dist = glm::length(d);
                if (vectorAngle(axis, d) <= angle + 1e-12 && dist <= radius + 1e-12) {
                    dist = glm::length(d);
                    int SDFindex = meshIndexTo1DIndex(x, y, z, xVals.size(), yVals.size());
                    if (dist < fabs(SDF[SDFindex])) {
                        //cout << "size: " << xSize*ySize*zSize << " index: " << SDFindex << endl;
                        SDF[SDFindex] = dir * dist;
                    }
                }

            }
        }
    }

}
