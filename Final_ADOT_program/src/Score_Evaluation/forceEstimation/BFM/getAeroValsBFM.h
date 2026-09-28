#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <mpi/mpi.h>

#include "getForcesBFM.h"
#include "../../../../compile_config.h"
#include "../../calculateScore.h"

//Gets the aerodynamic forces (net force, torque) on the aircraft for a single configuration
//Forces are normalised for velocity squared
//The first three columns of positionVariables is the vector of airflow, the next three are the gravity unit vector,
//any additional columns are angles ofcontrol surfaces
glm::dmat2x3 getAeroValsBFM(int numForceRegions, int numVelRegions,
    std::vector<glm::dvec3>& regionForces, std::vector<glm::dvec3>& regionTorques, 
    std::vector<double>& regionAvgVels, std::map<std::string, double> simParams, 
    int pos, int test, bool firstSim, int simParallelOpt, int nSimNodes, int nSimTasksPerNode);
