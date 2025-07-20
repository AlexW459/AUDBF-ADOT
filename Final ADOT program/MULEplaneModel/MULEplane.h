#pragma once

#include <vector>
#include <string>
#include <algorithm>

#include "../Mesh_Generation/profile.h"
#include "../aircraft.h"

void calcDerivedParams(vector<string>&, vector<double>&);

profile* fuselageProfile(vector<string> paramNames, vector<double> paramVals, double meshRes);

extrusionData extrudeFuselage(vector<string> paramNames, vector<double> paramVals, double meshRes);
