#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <regex>
#include <cmath>
#include <stdexcept>
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

#include "readForceFileBFM.h"
#include "readVelMagFileBFM.h"

//Returns net force and net torque on aircraft, excluding the tail. Forces on the tail are
//returned using reference arguments
std::pair<glm::dvec3, glm::dvec3> getForcesBFM(std::string filePath, int numForceRegions,
    int numVelRegions, std::vector<glm::dvec3>& regionForces, std::vector<glm::dvec3>& regionTorques,
    std::vector<double>& velRegionMags, double latestTime);