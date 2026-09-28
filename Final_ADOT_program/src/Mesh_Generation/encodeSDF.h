#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <stdexcept>
#include <fstream>

#include "../Score_Evaluation/calculateScore.h"
#include "../../compile_config.h"

void encodeSDF(std::vector<double> SDF, glm::dmat2x3 boundingBox, glm::dmat2x3 widerBox,
    glm::ivec3 SDFsize, glm::dvec3 COM, double boundingRadius, glm::ivec3 extraCells, 
    double CELL_GRADIENT, int pos, int test);