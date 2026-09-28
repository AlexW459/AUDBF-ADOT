#pragma once

#include <vector>
#include <string>
#include <algorithm>
#include <complex>


#include "../aircraft.h"
#include "../Mesh_Generation/profile.h"
#include "../Mesh_Generation/extrusionGen.h"
#include "../Mesh_Generation/MC.h"
#include "../readCSV.h"

using namespace std;

aircraft constructAircraft();

void calcDerivedParams(vector<string>& paramNames, vector<double>& paramVals, 
    const vector<dataTable>& discreteTables, vector<int> discreteVals, 
    vector<glm::dmat2x3>& velocityRegions);

double rateDesign(vector<string> fullParamNames, vector<double> fullParamVals, 
    vector<dataTable> discreteTables, vector<int> discreteVals, vector<vector<double>> positionVariables,
    double mass, vector<glm::dvec3> COMs, vector<glm::dmat3> MOIs, vector<glm::dvec3> totalForces, 
    vector<glm::dvec3> totalTorques, vector<vector<glm::dvec3>> regionForces,
    vector<vector<glm::dvec3>> regionTorques, vector<vector<double>> regionVelMags,
    vector<vector<glm::dvec3>> POIs, vector<glm::dvec3> partDirections);
    
//Finds the velocity at a given configuration
double calculateVelocity(vector<double> motorMaxThrusts, vector<double> motorMaxRPMs, 
    vector<double> motorPropPitches, double throttle, double dragCoeff, 
    vector<glm::dvec3> motorThrustDirs, double pitch);
    
std::vector<glm::dvec3> findIntersections(glm::dvec3 vert1, glm::dvec3 vert2, MC::mcMesh mesh);


profile fuselageProfile(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeFuselage(vector<string> paramNames, vector<double> paramVals, double meshRes);

profile wingProfile(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeRightWing(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeLeftWing(vector<string> paramNames, vector<double> paramVals, double meshRes);

profile motorPodProfile(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeRightMotorPod(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeLeftMotorPod(vector<string> paramNames, vector<double> paramVals, double meshRes);

profile empennageBoomProfile(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeEmpennageBoom(vector<string> paramNames, vector<double> paramVals, double meshRes);

profile horizontalStabiliserProfile(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeHorizontalStabiliser(vector<string> paramNames, vector<double> paramVals, double meshRes);

profile elevatorProfile(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeElevator(vector<string> paramNames, vector<double> paramVals, double meshRes);
