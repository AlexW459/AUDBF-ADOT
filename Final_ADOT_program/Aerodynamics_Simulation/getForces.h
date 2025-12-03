#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <regex>
#include <cmath>
#include <stdexcept>
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

//Returns net force and net torque on aircraft, in that order
std::pair<glm::dvec3, glm::dvec3> getForces(std::string filePath, std::string latestTime);
