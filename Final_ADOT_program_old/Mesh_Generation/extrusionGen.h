#pragma once

#include <vector>
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

#include "profile.h"
#include "../compile_config.h"



using namespace std;


struct extrusionData{
    //Description of extrusion
    vector<double> zSampleVals;
    vector<glm::dvec2> posVals;
    vector<glm::dvec2> scaleVals;

    //Description of location in model
    glm::dquat rotation;
    glm::dvec3 translation;
    glm::dvec3 pivotPoint;


    //Possible a control surface. Set controlAxis to (0, 0, 0) if not a control surface
    glm::dvec3 controlAxis;
    //double rotateAngle;

    //Possibly has a point mass inside
    vector<glm::dvec3> massLocations;
    vector<double> pointMasses;

    // Possibly facing a certain direction that will be recorded
    glm::dvec3 partDirection;

    // Whether to isolate forces on part
    bool isForceRegion;

    extrusionData(){};

    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, glm::dvec3 _controlAxis, vector<double> _pointMasses,
        vector<glm::dvec3> _massLocations, glm::dvec3 _partDirection, bool _isForceRegion)
        : zSampleVals(_zSampleVals), posVals(_posVals), 
        scaleVals(_scaleVals), rotation(_rotation), translation(_translation), pivotPoint(_pivotPoint), 
        controlAxis(_controlAxis), massLocations(_massLocations), 
        pointMasses(_pointMasses), partDirection(_partDirection), isForceRegion(_isForceRegion) {};

    //Barebones, basic part with no extras
    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint) : extrusionData(_zSampleVals, _posVals, _scaleVals, _rotation, 
        _translation, _pivotPoint, glm::dvec3(0.0), vector<double>(0), vector<glm::dvec3>(0),
        glm::dvec3(0.0), false) {};

    //Control surface but no point mass
    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, glm::dvec3 _controlAxis) : extrusionData(_zSampleVals, _posVals, 
        _scaleVals, _rotation, _translation, _pivotPoint, _controlAxis, vector<double>(0), 
        vector<glm::dvec3>(0), glm::dvec3(0.0), false) {};

    //Just has point mass
    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, vector<double> _pointMasses, vector<glm::dvec3> _massLocations) : 
        extrusionData(_zSampleVals, _posVals, _scaleVals, _rotation, _translation, _pivotPoint, 
        glm::dvec3(0.0), _pointMasses, _massLocations, glm::dvec3(0.0), false) {};

    //Not a control surface but has point mass and direction
    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, vector<double> _pointMasses, vector<glm::dvec3> _massLocations,
        glm::dvec3 _partDirection) : extrusionData(_zSampleVals, _posVals, _scaleVals, _rotation, _translation, _pivotPoint, 
        glm::dvec3(0.0), _pointMasses, _massLocations, _partDirection, false) {};

    //Not a control but has a forceRegion
    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, bool _isForceRegion) : extrusionData(_zSampleVals, _posVals, 
        _scaleVals, _rotation, _translation, _pivotPoint, glm::dvec3(0.0), vector<double>(0), 
        vector<glm::dvec3>(0), glm::dvec3(0.0), _isForceRegion) {};

    //Control and force region
    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, glm::dvec3 _controlAxis, bool _isForceRegion) : extrusionData(
        _zSampleVals, _posVals, _scaleVals, _rotation, _translation, _pivotPoint, _controlAxis,
        vector<double>(0), vector<glm::dvec3>(0), glm::dvec3(0.0), _isForceRegion) {};
    
    #ifdef USE_SDL
        void plot(int WINDOW_WIDTH, int WINDOW_HEIGHT, profile partProfile) const;
    #endif
};


int generateExtrusion(const profile& partProfile, const extrusionData& extrusion, vector<char>& adjMatrix, 
    vector<glm::dvec3>& points, glm::dmat2x3& boundingBox);

double getParam(string param, const vector<double>& paramVals, const vector<string>& paramNames);
