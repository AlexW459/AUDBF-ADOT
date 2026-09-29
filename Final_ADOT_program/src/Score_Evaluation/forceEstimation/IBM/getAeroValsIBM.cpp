#include "getAeroValsIBM.h"

using namespace std;

glm::dmat2x3 getAeroValsIBM(int numForceRegions, int numVelRegions,
    vector<glm::dvec3>& regionForces, vector<glm::dvec3>& regionTorques, 
    vector<double>& regionAvgVels, map<std::string, double> simParams, 
    int pos, int test, bool firstSim, int simParallelOpt, int nSimNodes, int nSimTasksPerNode){

    vector<glm::dvec3> totalForces, totalTorques;

    string caseDir = "Aerodynamics_Simulation_IBM_Test_" + to_string(test);

    //Runs simulation
    double endTime = firstSim ? simParams["SIMULATION_LENGTH_INITIAL"] : simParams["SIMULATION_LENGTH"];
    double deltaT = simParams.at("SIMULATION_DELTA_T");
    double writeInterval = simParams["SIMULATION_WRITE_INTERVAL"];
    string simScriptCall = string(projectRoot) + "/src/simScripts/IBM/runSim.sh " + to_string(endTime) + " " + 
        to_string(deltaT) + " " + to_string((int)writeInterval) + " " + to_string(pos) + " " +
        to_string(test) + " " + to_string(simParallelOpt) + " " 
        + to_string(nSimNodes) + " " + to_string(nSimTasksPerNode) +  " \"" + string(projectRoot) + "\" \"" +
        OPENFOAM_SOURCE + "\"";
    int failure = system(simScriptCall.c_str());
    if(failure) throw std::runtime_error("Runnning simulation failed");

    pair<glm::dvec3, glm::dvec3> forceVals = getForcesIBM(caseDir + "/", endTime);

    return glm::dmat2x3(forceVals.first, forceVals.second);
}

