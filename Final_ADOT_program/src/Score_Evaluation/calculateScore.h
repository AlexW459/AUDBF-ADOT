#pragma once

#include <vector>
#include <array>
#include <iostream>
#include <mpi/mpi.h>
#include <chrono>

#include "testModel/testModel.h"
#include "../Mesh_Generation/extrusion/extrusion.h"
#include "../Mesh_Generation/marchingCubes/marchingCubes.hpp"
#include "physEstimation/getVolVals.h"
#include "../Mesh_Generation/surface/initSDF.h"
#include "../Mesh_Generation/surface/genPartSDF.h"
#include "../Mesh_Generation/surface/SDFunion.h"
#include "../Mesh_Generation/generateOFmesh.h"
#include "../Mesh_Generation/encodeSDF.h"
#include "forceEstimation/BFM/getAeroValsBFM.h"
#include "forceEstimation/IBM/getAeroValsIBM.h"

#include "../../optim/header_only_version/optim.hpp"

constexpr const char* projectRoot = PROJECT_ROOT;

// MPI mesage types
enum MPI_COMMAND_TAGS{
    EXIT_PROGRAM,
    RUN_TEST,
    DATA_TO_HELPER,
    DATA_TO_HEAD
};

class ScoreEvaluator{
  public:
    /*ScoreFunc parameters are: configuration variables (AOA, elevator, throttle), 
  aerodynamic forces, velocity, oscillation frequency, damping coefficient, 
  dMdalpha, mass, paramNames, paramVals*/
  static void setSimParams(testModel _model, std::map<std::string, double> _simParams, 
    bool _writeObjs, int _simParallelOpt, int _nSimNodes, int _nSimTasksPerNode, int _procRank,
    int _nProcs);

  static double calculateScore(const optim::ColVec_t& paramVals, optim::ColVec_t *gradOut,
    void *optData);

  static void dealloc();

  

  private:
    static int modelNum;
    static testModel model;
    static std::map<std::string, double> simParams;
    static bool writeObjs;
    static int procRank;
    static int nProcs;
    static int nSimNodes;
    static int simParallelOpt;
    static int nSimTasksPerNode;  
    //static bool runTest; // Set to true when rank has finished all tasksx
        
};

