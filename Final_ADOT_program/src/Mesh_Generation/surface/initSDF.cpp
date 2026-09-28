#include "initSDF.h"

using namespace std;

glm::ivec3 initSDF(vector<double>& SDF, vector<double>& xVals, vector<double>& yVals,
    vector<double>& zVals, glm::dmat2x3& totalBoundingBox, double surfMeshRes){

    //Generates matrices of values
    glm::dvec3 boundSize = totalBoundingBox[1] - totalBoundingBox[0];

    //cout << "boundSize: " << boundSize[0] << ", " << boundSize[1] << ", " << boundSize[1] << endl;

    //Adjusts boundary to account for the fact that the resolution doesn't perfectly divide into
    //the bounding box
    double interval = 1.0/surfMeshRes;
    glm::dvec3 endBoundRemainder(fmod(boundSize[0], interval), fmod(boundSize[1], interval), fmod(boundSize[2], interval));

    //cout << "interval: " << interval << endl;
    //cout << "bound size: " << boundSize[0] << ", " << boundSize[1] << ", " << boundSize[2] << endl;
    //cout << "remainder: " << endBoundRemainder[0] << ", " << endBoundRemainder[1] << ", " << endBoundRemainder[2] << endl;

    // Adds on additional point so that there is a grid point at both the start and end bounds of the interval
    glm::ivec3 SDFsize = glm::ivec3(floor(boundSize/interval)) + glm::ivec3(1);
    // Extends bounds so that the bounds are a multiple of the interval
    if(endBoundRemainder[0] > 1e-10 && endBoundRemainder[0] < interval - 1e-10){
        totalBoundingBox[1][0] += interval - endBoundRemainder[0];
        SDFsize[0] += 1;
    }
    if(endBoundRemainder[1] > 1e-10 && endBoundRemainder[1] < interval - 1e-10){
        totalBoundingBox[1][1] += interval - endBoundRemainder[1];
        SDFsize[1] += 1;
    }
    if(endBoundRemainder[2] > 1e-10 && endBoundRemainder[1] < interval - 1e-10){
        totalBoundingBox[1][2] += interval - endBoundRemainder[2];
        SDFsize[2] += 1;
    }

    boundSize = totalBoundingBox[1] - totalBoundingBox[0];

    int totalSDFSize = SDFsize[0]*SDFsize[1]*SDFsize[2];
    //cout << "total SDF size: " << totalSDFSize << endl;

    //Fills arrays
    xVals.resize(SDFsize[0]);
    yVals.resize(SDFsize[1]);
    zVals.resize(SDFsize[2]);
    for(int i = 0; i < SDFsize[0]; i++){
        xVals[i] = totalBoundingBox[0][0] + i*interval;
    }
    for(int i = 0; i < SDFsize[1]; i++){
        yVals[i] = totalBoundingBox[0][1] + i*interval;
    }
    for(int i = 0; i < SDFsize[2]; i++){
        zVals[i] = totalBoundingBox[0][2] + i*interval;
    }

    SDF.resize(totalSDFSize, INF);

    //cout << "total bounding box: " << totalBoundingBox[0][0] << ", " << totalBoundingBox[0][1] << ", " << totalBoundingBox[0][2] << " - "
    //        << totalBoundingBox[1][0] << ", " << totalBoundingBox[1][1] << ", " << totalBoundingBox[1][2] << endl;
    
    //cout << "SDF size: " << xVals.size() << " " << yVals.size() << " " << zVals.size() << endl;

    return SDFsize;
}