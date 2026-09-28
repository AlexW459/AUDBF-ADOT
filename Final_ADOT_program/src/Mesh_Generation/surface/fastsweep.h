#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "SDFutils/meshIndexTo1DIndex.h"
#include "../extrusion/extrusion.h"

void fastSweep(std::vector<double>& field, int xSize, int ySize, int zSize, double h, int nSweeps);

