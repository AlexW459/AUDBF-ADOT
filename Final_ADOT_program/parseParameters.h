#pragma once

#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <regex>
#include <fstream>

using namespace std;

enum parallelOpt{
    NOT_PARALLEL,
    PARALLEL,
    PARALLEL_SLURM
};

void parseParameters(int argc, char *argv[], string parameterFileName, int& meshParallelOpt,
    int& simParallelOpt, int& nSimNodes, int& nSimTasksPerNode, vector<double>& simParameters,
    bool& writeObjs);