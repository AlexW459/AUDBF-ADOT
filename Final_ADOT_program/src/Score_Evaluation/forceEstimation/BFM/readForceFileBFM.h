#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <regex>
#include <cmath>
#include <stdexcept>
#include <glm/glm.hpp>

std::pair<glm::dvec3, glm::dvec3> readForceFileBFM(std::string fileName, double latestTime);
