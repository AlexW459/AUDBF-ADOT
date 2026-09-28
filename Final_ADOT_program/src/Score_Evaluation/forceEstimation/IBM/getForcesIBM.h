#pragma once

#include <regex>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>

std::pair<glm::dvec3, glm::dvec3> getForcesIBM(std::string filePath, double latestTime);