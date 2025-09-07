#pragma once

#include <vector>
#include <string>
#include <algorithm>

#include "../Mesh_Generation/profile.h"
#include "../aircraft.h"

void calcDerivedParams(vector<string>& paramNames, vector<double>& paramVals);

profile fuselageProfile(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeFuselage(vector<string> paramNames, vector<double> paramVals, double meshRes);

profile wingProfile(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeRightWing(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeLeftWing(vector<string> paramNames, vector<double> paramVals, double meshRes);
