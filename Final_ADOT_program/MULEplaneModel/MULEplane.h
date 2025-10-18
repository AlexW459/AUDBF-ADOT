#pragma once

#include <vector>
#include <string>
#include <algorithm>

#include "../Mesh_Generation/profile.h"
#include "../aircraft.h"

void calcDerivedParams(vector<string>& paramNames, vector<double>& paramVals, 
    const vector<dataTable>& discreteTables, vector<int> discreteVals);

profile fuselageProfile(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeFuselage(vector<string> paramNames, vector<double> paramVals, double meshRes);

profile wingProfile(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeRightWing(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeLeftWing(vector<string> paramNames, vector<double> paramVals, double meshRes);

profile motorPodProfile(vector<string> paramNames, vector<double> paramVals, double meshRes);

profile elevatorProfile(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeRightElevator(vector<string> paramNames, vector<double> paramVals, double meshRes);