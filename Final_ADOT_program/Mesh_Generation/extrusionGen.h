#pragma once

#include <vector>
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

#include "profile.h"

using namespace std;

struct extrusionData{
    vector<double> zSampleVals;
    vector<glm::dvec2> posVals;
    vector<glm::dvec2> scaleVals;

    glm::dquat rotation;
    glm::dvec3 translation;
    glm::dvec3 pivotPoint;

    extrusionData(){};

    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint):zSampleVals(_zSampleVals),
        posVals(_posVals), scaleVals(_scaleVals), rotation(_rotation), 
        translation(_translation), pivotPoint(_pivotPoint){};
};

int generateExtrusion(const profile& partProfile, const extrusionData& extrusion, vector<char>& adjMatrix, 
    vector<glm::dvec3>& points, glm::dmat2x3& boundingBox);