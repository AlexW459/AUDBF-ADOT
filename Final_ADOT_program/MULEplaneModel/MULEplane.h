#pragma once

#include <vector>
#include <string>
#include <algorithm>
#include <glm/glm.hpp>
#include <complex>


#include "../ADOT.h"

extern "C" testModel constructModel();

extern "C" void calcDerivedParams(std::vector<std::string>& paramNames, std::vector<double>& paramVals, 
    const std::vector<dataTable>& discreteTables, std::vector<glm::dmat2x3>& velocityRegions);

extern "C" double rateDesign(std::vector<std::string> fullParamNames, std::vector<double> fullParamVals, 
    std::vector<std::vector<double>> positionVariables, double mass, std::vector<glm::dvec3> COMs, 
    std::vector<glm::dmat3> MOIs, std::vector<glm::dvec3> totalForces, 
    std::vector<glm::dvec3> totalTorques, std::vector<std::vector<glm::dvec3>> regionForces,
    std::vector<std::vector<glm::dvec3>> regionTorques, std::vector<std::vector<double>> regionVelMags,
    std::vector<std::vector<glm::dvec3>> POIs, std::vector<glm::dvec3> partDirections);
    
//Finds the velocity at a given configuration
double calculateVelocity(std::vector<double> motorMaxThrusts, std::vector<double> motorMaxRPMs, 
    std::vector<double> motorPropPitches, double throttle, double dragCoeff, 
    std::vector<glm::dvec3> motorThrustDirs, double pitch);
    
std::vector<glm::dvec3> findIntersections(glm::dvec3 vert1, glm::dvec3 vert2, MC::mcMesh mesh);

extern "C" profile fuselageProfile(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);

extern "C" extrusionData extrudeFuselage(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);

extern "C" profile wingProfile(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);

extern "C" extrusionData extrudeRightWing(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);

extern "C" extrusionData extrudeLeftWing(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);

extern "C" profile motorPodProfile(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);

extern "C" extrusionData extrudeRightMotorPod(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);

extern "C" extrusionData extrudeLeftMotorPod(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);

extern "C" profile empennageBoomProfile(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);

extern "C" extrusionData extrudeEmpennageBoom(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);

extern "C" profile horizontalStabiliserProfile(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);

extern "C" extrusionData extrudeHorizontalStabiliser(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);

extern "C" profile elevatorProfile(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);

extern "C" extrusionData extrudeElevator(std::vector<std::string> paramNames, std::vector<double> paramVals, double meshRes);
