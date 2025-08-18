#pragma once

#include <vector>
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

#include "profile.h"
#include "extrusionGen.h"

using namespace std;

//Gets SDF of entire aircraft. Returns dimensions of SDF
glm::vec3 generateSDF(vector<float>& SDF, vector<profile>& profiles, vector<int> profileIndices,
    vector<extrusionData>& extrusions, vector<vector<int>> parentIndices, glm::mat2x3 totalBoundingBox, 
    vector<glm::mat2x3> boundingBoxes, double surfMeshRes);

//Gets SDF of a single part with a specified resolution. Returns size of SDF
glm::ivec3 generatePartSDF(const vector<extrusionData>& extrusions, const profile& partProfile, int partIndex,
    vector<int> parentIndices, const vector<glm::vec3>& meshGrid, glm::ivec3 SDFSize, 
    glm::imat2x3 boundingIndices, double surfMeshRes, vector<float>& SDF);


void applyGaussianBlur(float sigma, int n, vector<float>& SDF, glm::ivec3 SDFSize);