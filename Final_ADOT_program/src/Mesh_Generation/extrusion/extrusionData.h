#pragma once

#include <vector>
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "../profile/profile.h"
#include "../../../compile_config.h"


struct extrusionData{
    //Description of extrusion
    std::vector<double> zVals;
    std::vector<glm::dvec2> posVals;
    std::vector<glm::dvec2> scaleVals;

    //Description of location in model
    glm::dquat rotation;
    glm::dvec3 translation;
    glm::dvec3 pivotPoint;


    //Possible a control surface. Set controlAxis to (0, 0, 0) if not a control surface
    glm::dvec3 controlAxis;
    //double rotateAngle;

    //Possibly has a point mass inside
    std::vector<glm::dvec3> massLocations;
    std::vector<double> pointMasses;

    // Possibly facing a certain direction that will be recorded
    glm::dvec3 partDirection;

    // Whether to isolate forces on part
    bool isForceRegion;

    extrusionData(){};

    extrusionData(std::vector<double> _zVals, std::vector<glm::dvec2> _posVals,
        std::vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, glm::dvec3 _controlAxis, std::vector<double> _pointMasses,
        std::vector<glm::dvec3> _massLocations, glm::dvec3 _partDirection, bool _isForceRegion)
        : zVals(_zVals), posVals(_posVals), 
        scaleVals(_scaleVals), rotation(_rotation), translation(_translation), pivotPoint(_pivotPoint), 
        controlAxis(_controlAxis), massLocations(_massLocations), 
        pointMasses(_pointMasses), partDirection(_partDirection), isForceRegion(_isForceRegion) {};

    //Barebones, basic part with no extras
    extrusionData(std::vector<double> _zVals, std::vector<glm::dvec2> _posVals,
        std::vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint) : extrusionData(_zVals, _posVals, _scaleVals, _rotation, 
        _translation, _pivotPoint, glm::dvec3(0.0), std::vector<double>(0), std::vector<glm::dvec3>(0),
        glm::dvec3(0.0), false) {};

    //Control surface but no point mass
    extrusionData(std::vector<double> _zVals, std::vector<glm::dvec2> _posVals,
        std::vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, glm::dvec3 _controlAxis) : extrusionData(_zVals, _posVals, 
        _scaleVals, _rotation, _translation, _pivotPoint, _controlAxis, std::vector<double>(0), 
        std::vector<glm::dvec3>(0), glm::dvec3(0.0), false) {};

    //Just has point mass
    extrusionData(std::vector<double> _zVals, std::vector<glm::dvec2> _posVals,
        std::vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, std::vector<double> _pointMasses, std::vector<glm::dvec3> _massLocations) : 
        extrusionData(_zVals, _posVals, _scaleVals, _rotation, _translation, _pivotPoint, 
        glm::dvec3(0.0), _pointMasses, _massLocations, glm::dvec3(0.0), false) 
        {};

    //Not a control surface but has point mass and direction
    extrusionData(std::vector<double> _zVals, std::vector<glm::dvec2> _posVals,
        std::vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, std::vector<double> _pointMasses, std::vector<glm::dvec3> _massLocations,
        glm::dvec3 _partDirection) : extrusionData(_zVals, _posVals, _scaleVals, _rotation, _translation, 
        _pivotPoint, glm::dvec3(0.0), _pointMasses, _massLocations, _partDirection, false) 
        {};


    //Not a control but has a forceRegion
    extrusionData(std::vector<double> _zVals, std::vector<glm::dvec2> _posVals,
        std::vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, bool _isForceRegion) : extrusionData(_zVals, _posVals, 
        _scaleVals, _rotation, _translation, _pivotPoint, glm::dvec3(0.0), std::vector<double>(0), 
        std::vector<glm::dvec3>(0), glm::dvec3(0.0), _isForceRegion) {};

    //Control and force region
    extrusionData(std::vector<double> _zVals, std::vector<glm::dvec2> _posVals,
        std::vector<glm::dvec2> _scaleVals, glm::dquat _rotation, glm::dvec3 _translation, 
        glm::dvec3 _pivotPoint, glm::dvec3 _controlAxis, bool _isForceRegion) : extrusionData(
        _zVals, _posVals, _scaleVals, _rotation, _translation, _pivotPoint, _controlAxis,
        std::vector<double>(0), std::vector<glm::dvec3>(0), glm::dvec3(0.0), _isForceRegion) {};
    
};

