#pragma once

#include <functional>
#include <vector>
#include <string>
#include <glm/glm.hpp>

#include "dataTable.h"

typedef std::function<void(std::vector<std::string>& paramNames, std::vector<double>& paramVals, 
    const std::vector<dataTable>& discreteTables, std::vector<glm::dmat2x3>& velRegions)> derParamFuncType;