#pragma once

#include <vector>
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

#include "profile.h"
#include "extrusionGen.h"

#define INITIAL_GEN true
#define UPDATE_GEN false

using namespace std;

int meshIndexTo1DIndex(int i, int j, int k, int sizeX, int sizeY);

//Initialises SDF. Returns dimensions of SDF
glm::ivec3 initSDF(vector<double>& SDF, vector<glm::dvec3>& XYZ, glm::dmat2x3 totalBoundingBox, double surfMeshRes);


//Returns updated SDF with all specified parts added
vector<double> updateSDF(vector<double> SDF, glm::ivec3 SDFSize, const vector<glm::dvec3>& XYZ, 
    const vector<profile>& profiles, vector<int> profileIndices, const vector<extrusionData>& extrusions, 
    const vector<vector<int>>& parentIndices, const vector<glm::dmat2x3>& boundingBoxes,
     glm::dmat2x3 totalBoundingBox, vector<int> meshSurfaces, double surfMeshRes);

//Gets SDF of a single part with a specified resolution. Returns size of SDF
glm::ivec3 generatePartSDF(const vector<extrusionData>& extrusions, const profile& partProfile, int partIndex,
    vector<int> parentIndices, const vector<glm::dvec3>& meshGrid, glm::ivec3 SDFSize, 
    glm::imat2x3 boundingIndices, double surfMeshRes, vector<double>& SDF);


//void applyGaussianBlur(float sigma, int n, vector<double>& SDF, glm::ivec3 SDFSize);