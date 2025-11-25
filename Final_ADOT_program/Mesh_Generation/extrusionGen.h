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

    bool isControl;
    glm::dvec3 controlAxis = glm::dvec3(0.0, 0.0, 0.0);
    double rotateAngle;

    glm::dvec3 massLocation;
    double pointMass;

    extrusionData(){};

    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, bool _isControl, glm::dvec3 _controlAxis, double _pointMass,
        glm::dvec3 _massLocation) : zSampleVals(_zSampleVals), posVals(_posVals), scaleVals(_scaleVals), 
        rotation(_rotation), translation(_translation), pivotPoint(_pivotPoint), isControl(_isControl), 
        controlAxis(_controlAxis), massLocation(_massLocation), pointMass(_pointMass) {};

    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint) : extrusionData(_zSampleVals, _posVals, _scaleVals, _rotation, 
            _translation, _pivotPoint, false, glm::dvec3(0.0, 0.0, 0.0), 0.0, glm::dvec3(0.0, 0.0, 0.0)) {};

    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, glm::dvec3 _controlAxis) : extrusionData(_zSampleVals, _posVals, _scaleVals, _rotation, 
            _translation, _pivotPoint, true, _controlAxis, 0.0, glm::dvec3(0.0, 0.0, 0.0)) {};

    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, double _pointMass,
        glm::dvec3 _massLocation) : extrusionData(_zSampleVals, _posVals, _scaleVals, _rotation, 
            _translation, _pivotPoint, false, glm::dvec3(0.0, 0.0, 0.0), _pointMass, _massLocation) {};

};

int generateExtrusion(const profile& partProfile, const extrusionData& extrusion, vector<char>& adjMatrix, 
    vector<glm::dvec3>& points, glm::dmat2x3& boundingBox);

double getParam(string param, const vector<double>& paramVals, const vector<string>& paramNames);