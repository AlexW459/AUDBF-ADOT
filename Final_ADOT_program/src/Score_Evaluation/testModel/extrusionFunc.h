#pragma once

#include <functional>
#include <vector>
#include <string>

#include "../../Mesh_Generation/extrusion/extrusionData.h"

typedef std::function<extrusionData(std::vector<std::string>, std::vector<double>, double)> 
    extrusionFuncType;