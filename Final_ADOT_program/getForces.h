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

//Returns net force and net torque on aircraft, excluding the tail. Forces on the tail are
//returned using reference arguments
std::pair<glm::dvec3, glm::dvec3> getForces(std::string filePath, double& tailVelMag, 
    glm::dvec3& tailForce, glm::dvec3& tailTorque, double latestTime);
