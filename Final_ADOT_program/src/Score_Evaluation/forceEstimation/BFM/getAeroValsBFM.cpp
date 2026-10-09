#include "getAeroValsBFM.h"

using namespace std;

glm::dmat2x3 getAeroValsBFM(int numForceRegions, int numVelRegions,
    vector<glm::dvec3>& regionForces, vector<glm::dvec3>& regionTorques, 
    vector<double>& regionAvgVels, map<string, double> simParams, 
    int pos, int test, bool firstSim, int simParallelOpt, int nSimNodes, int nSimTasksPerNode){

    vector<glm::dvec3> totalForces, totalTorques;

    string caseDir = "Aerodynamics_Simulation_BFM_Test_" + to_string(test);


    //Runs simulation
    double endTime = firstSim ? simParams["SIMULATION_LENGTH_INITIAL"] : simParams["SIMULATION_LENGTH"];
    double deltaT = simParams.at("SIMULATION_DELTA_T");
    double writeInterval = simParams["SIMULATION_WRITE_INTERVAL"];
    string simScriptCall = string(projectRoot) + "/src/simScripts/BFM/runSim.sh " + 
        to_string(endTime) + " " + to_string(deltaT) + " " + to_string(writeInterval) + " " + 
        to_string(pos) + " " + to_string(test) + " " + to_string(simParallelOpt) + " " 
        + to_string(nSimNodes) + " " + to_string(nSimTasksPerNode) + " \"" + OPENFOAM_SOURCE + "\"";
    int failure = system(simScriptCall.c_str());
    if(failure) throw std::runtime_error("Runnning simulation failed");


    //Get aerodynamic forces
    pair<glm::dvec3, glm::dvec3> forceVals;
    forceVals = getForcesBFM(caseDir + "/", numForceRegions, numVelRegions, regionForces,
        regionTorques, regionAvgVels,  endTime);

    /*cout << "Force and torque from simulation test=" << test << ", pos= " << pos << ": " << 
        "(" << forceVals.first[0] << ", " << forceVals.first[1] << ", " <<
        forceVals.first[2] << ") (" << forceVals.second[0] << ", " << 
        forceVals.second[1] << ", " << forceVals.second[2] << ")" << endl;*/

    return glm::dmat2x3(forceVals.first, forceVals.second);
}