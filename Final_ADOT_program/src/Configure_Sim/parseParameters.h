#pragma once

#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <regex>
#include <fstream>

#include "loadObjectFile.h"
#include "../main.h"

enum parallelOpt{
    NOT_PARALLEL,
    PARALLEL,
    PARALLEL_SLURM
};

void parseParameters(int argc, char *argv[], std::string parameterFileName, int& meshParallelOpt,
    int& simParallelOpt, int& nSimNodes, int& nSimTasksPerNode, std::map<std::string, double>& simParameters,
    bool& writeObjs, optim::algo_settings_t& optimSettings, void*& constructModelFunc, void*& objFileHandle);