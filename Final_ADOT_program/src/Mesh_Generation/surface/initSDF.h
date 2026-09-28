#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "../extrusion/extrusion.h"
#include "SDFutils/meshIndexTo1DIndex.h"


//Initialises SDF grid
glm::ivec3 initSDF(std::vector<double>& SDF, std::vector<double>& xVals, std::vector<double>& yVals,
    std::vector<double>& zVals, glm::dmat2x3& totalBoundingBox, double surfMeshRes);
