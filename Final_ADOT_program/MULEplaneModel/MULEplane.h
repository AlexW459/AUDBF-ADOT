#pragma once

#include <vector>
#include <string>
#include <algorithm>
#include <complex>

#include "../Mesh_Generation/profile.h"
#include "../aircraft.h"

void calcDerivedParams(vector<string>& paramNames, vector<double>& paramVals, 
    const vector<dataTable>& discreteTables, vector<int> discreteVals, double& wingRootChord, 
    double& wingLength, double& wingScale, double& tailHorizArea);

double rateDesign(array<double, 3> bestConfig, double velocity, glm::dvec3 aeroForces, double oscillationFreq, double dampingCoeff,
    double dMdAlpha, double mass, vector<string> fullParamNames, vector<double> fullParamVals);
    
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
