#pragma once

#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <iostream>
#include <stdexcept>

#include "../../compile_config.h"
#include "../Score_Evaluation/calculateScore.h"

// Prepares and runs snappy hex mesh
void generateOFmesh(std::vector<double> posVals, glm::dmat2x3 boundingBox, glm::dmat2x3 widerBox,
    glm::dvec3 pointInMesh, glm::ivec3 boxSize, glm::ivec3 extraCells, double CELL_GRADIENT,
    std::vector<glm::dmat2x3> forceRegions, std::vector<glm::dmat2x3> velRegions, int position, int test);