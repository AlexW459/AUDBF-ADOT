#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "SDFutils/pointPosToIndex.h"
#include "SDFutils/meshIndexTo1DIndex.h"
#include "initSDF.h"

void faceSDF(glm::dvec3 pts[6], int dir, const std::vector<double>& xVals, const std::vector<double>& yVals,
    const std::vector<double>& zVals, std::vector<double>& SDF);
