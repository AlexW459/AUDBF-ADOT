#pragma once

#include <vector>
#include <string>
#include <functional>
#include <glm/glm.hpp>

#include "dataTable.h"

typedef std::function<double(std::vector<std::string> fullParamNames, std::vector<double> fullParamVals, 
    std::vector<std::vector<double>> positionVariables, double mass, std::vector<glm::dvec3> COMs,
    std::vector<glm::dmat3> MOIs, std::vector<glm::dvec3> totalForces, 
    std::vector<glm::dvec3> totalTorques, std::vector<std::vector<glm::dvec3>> regionForces,
    std::vector<std::vector<glm::dvec3>> regionTorques, std::vector<std::vector<double>> regionVelMags,
    std::vector<std::vector<glm::dvec3>> POIs, std::vector<glm::dvec3> partDirections)> scoreFuncType;