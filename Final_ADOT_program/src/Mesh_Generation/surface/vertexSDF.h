#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "SDFutils/meshIndexTo1DIndex.h"
#include "SDFutils/vectorAngle.h"
#include "SDFutils/pointPosToIndex.h"
#include "initSDF.h"

void vertexSDF(glm::dvec3 vert, double radius, glm::dvec3 axis, double angle, int dir, 
    const std::vector<double>& xVals, const std::vector<double>& yVals, 
    const std::vector<double>& zVals, std::vector<double>& SDF);
