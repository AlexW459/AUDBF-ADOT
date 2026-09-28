#pragma once

#include <vector>
#include <glm/glm.hpp>

#include "../extrusion/extrusion.h"


// Sets first SDF to a union of the two SDFs
void SDFunion(std::vector<double>& initialSDF, const std::vector<double> secondarySDF);
