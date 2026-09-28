#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <regex>
#include <cmath>
#include <stdexcept>
#include <glm/glm.hpp>

double readVelMagFileBFM(std::string filePath, double latestTime);