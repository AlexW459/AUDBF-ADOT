#pragma once

#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <mpi/mpi.h>

#include "getForcesIBM.h"
#include "../../../../compile_config.h"
#include "../../calculateScore.h"

glm::dmat2x3 getAeroValsIBM(int numForceRegions, int numVelRegions,
    std::vector<glm::dvec3>& regionForces, std::vector<glm::dvec3>& regionTorques, 
    std::vector<double>& regionAvgVels, std::map<std::string, double> simParams,
    int pos, int test, bool firstSim, int simParallelOpt, int nSimNodes, int nSimTasksPerNode);


