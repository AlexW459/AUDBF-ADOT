#pragma once

#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <iostream>
#include <stdexcept>

#include "../../compile_config.h"
#include "../Score_Evaluation/calculateScore.h"

// Prepares and runs snappy hex mesh
void generateOFmesh(std::vector<double> posVals, glm::dmat2x3 boundingBox, glm::dvec3 pointInMesh,
    std::vector<glm::dmat2x3> forceRegions, std::vector<glm::dmat2x3> velRegions, int position, int test);