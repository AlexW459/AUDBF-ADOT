#pragma once

#include <vector>
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

#include "profile.h"
#include "../constants.h"



using namespace std;

struct motorData{
    glm::dvec3  thrustDir;
    glm::dvec3  propPos;
    double      maxThrust; //Newtons
    double      maxRPM;
    double      propPitch; //Inches

    motorData(glm::dvec3 _thrustDir, glm::dvec3 _propPos, double _maxThrust,
        double _maxRPM, double _propPitch) : thrustDir(_thrustDir), propPos(_propPos),
        maxThrust(_maxThrust), maxRPM(_maxRPM), propPitch(_propPitch) {};

    motorData() : thrustDir(glm::dvec3(0.0, 0.0, 0.0)), propPos(glm::dvec3(0.0, 0.0, 0.0)),
        maxThrust(0.0), maxRPM(1.0), propPitch(1.0){};
};

struct extrusionData{
    //Description of extrusion
    vector<double> zSampleVals;
    vector<glm::dvec2> posVals;
    vector<glm::dvec2> scaleVals;

    //Description of location in model
    glm::dquat rotation;
    glm::dvec3 translation;
    glm::dvec3 pivotPoint;


    //Possible a control surface
    bool isControl;
    glm::dvec3 controlAxis;
    double rotateAngle;

    //Possibly has a point mass inside
    glm::dvec3 massLocation;
    double pointMass;

    //Possibly is a motor with propellor
    bool isMotor;
    motorData motor;

    //Possibly is a horizontal part of the tail
    bool isHorizontalStabiliser;


    extrusionData(){};

    //General constructor
    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, bool _isControl, glm::dvec3 _controlAxis, double _pointMass,
        glm::dvec3 _massLocation, bool _isMotor, motorData _motor, bool _isHorizontalStabiliser)
        : zSampleVals(_zSampleVals), posVals(_posVals), 
        scaleVals(_scaleVals), rotation(_rotation), translation(_translation), pivotPoint(_pivotPoint), 
        isControl(_isControl), controlAxis(_controlAxis), rotateAngle(0.0), massLocation(_massLocation), 
        pointMass(_pointMass), isMotor(_isMotor), motor(_motor), 
        isHorizontalStabiliser(_isHorizontalStabiliser) {};

    //Barebones, basic part with no extras
    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint) : extrusionData(_zSampleVals, _posVals, _scaleVals, _rotation, 
        _translation, _pivotPoint, false, glm::dvec3(0.0), 0.0, glm::dvec3(0.0),
        false, motorData(), false) {};

    //Control surface but no point mass
    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, glm::dvec3 _controlAxis) : 
        extrusionData(_zSampleVals, _posVals, _scaleVals, _rotation, 
        _translation, _pivotPoint, true, _controlAxis, 0.0, glm::dvec3(0.0, 0.0, 0.0), 
        false, motorData(), false) {};

    //Not a control surface but has point mass
    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, double _pointMass, glm::dvec3 _massLocation) : 
        extrusionData(_zSampleVals, _posVals, _scaleVals, _rotation, 
        _translation, _pivotPoint, false, glm::dvec3(0.0), _pointMass, _massLocation,
        false, motorData(), false) {};

    //Not a control but has a motor
    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, double _pointMass, glm::dvec3 _massLocation,
        motorData _motor) : extrusionData(_zSampleVals, _posVals, _scaleVals, _rotation, 
        _translation, _pivotPoint, false, glm::dvec3(0.0), _pointMass, _massLocation,
        true, _motor, false) {};

    //Horizontal Stabiliser
    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, bool _isHorizontalStabiliser) : extrusionData(_zSampleVals, 
        _posVals, _scaleVals, _rotation, _translation, _pivotPoint, false, 
        glm::dvec3(0.0), 0.0, glm::dvec3(0.0),
        false, motorData(), _isHorizontalStabiliser) {};

    //Elevator
    extrusionData(vector<double> _zSampleVals, vector<glm::dvec2> _posVals,
        vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, glm::dvec3 _controlAxis, bool _isHorizontalStabiliser) : 
        extrusionData(_zSampleVals, _posVals, _scaleVals, _rotation, 
        _translation, _pivotPoint, true, _controlAxis, 0.0, glm::dvec3(0.0, 0.0, 0.0), 
        false, motorData(), _isHorizontalStabiliser) {};

    #ifdef USE_SDL
        void plot(int WINDOW_WIDTH, int WINDOW_HEIGHT, profile partProfile) const;
    #endif
};


int generateExtrusion(const profile& partProfile, const extrusionData& extrusion, vector<char>& adjMatrix, 
    vector<glm::dvec3>& points, glm::dmat2x3& boundingBox);

double getParam(string param, const vector<double>& paramVals, const vector<string>& paramNames);
