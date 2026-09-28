#include "getForcesBFM.h"

using namespace std;

pair<glm::dvec3, glm::dvec3> getForcesBFM(string filePath, int numForceRegions,
    int numVelRegions, vector<glm::dvec3>& regionForces, vector<glm::dvec3>& regionTorques, 
    vector<double>& velRegionMags, double latestTime){


    //Regex expressiosn for single line of file.
    //First group is time, the next three are components of pressure force, 
    //the next three are components of viscous force, the next three are
    //components of pressure torque, the next three are components of viscous torque
    //regex forcesMatch("(\\d+\\.\\d+) +\\t\\(\\((-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\) \\((-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\)\\) \\(\\((-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\) \\((-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\)\\)");
    //0.002             	((8.8026221108e+00 -9.4824053925e-01 1.8044495585e+01) (-2.5786969248e-03 -1.1765188435e-03 9.9951061308e-02)) ((-2.9949115575e-01 -9.4618962994e+00 6.5449899611e-01) (2.7422164643e-04 -9.6754994108e-03 8.6147374731e-05))
    //regex velMagMatch("(\\d\\.\\d+) +\\t?(\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d)");
    //0.296             	3.3883516015e+01
    
    //Opens force file for body of aircraft
    string mainForceFileName = "postProcessing/aeroForces/0/forces.dat";
    pair<glm::dvec3, glm::dvec3> totalForces = readForceFileBFM(filePath + mainForceFileName, latestTime);
    glm::dvec3 totalForce = totalForces.first;
    glm::dvec3 totalTorque = totalForces.second;

    //Gets force from each region
    regionForces.resize(numForceRegions);
    regionTorques.resize(numForceRegions);
    for(int i = 0; i < numForceRegions; i++){
        string forceFileName = "postProcessing/aeroForces_ " +  to_string(i) + "/0/forces.dat";
        pair<glm::dvec3, glm::dvec3> localForces = readForceFileBFM(filePath + forceFileName, latestTime);
        totalForce += localForces.first;
        totalTorque += localForces.second;

        regionForces[i] = localForces.first;
        regionTorques[i] = localForces.second;
    }

    //Gets velocity magnitudes from each region
    for(int i = 0; i < numVelRegions; i++){
        string velMagFileName = "postProcessing/velocity_" + to_string(i) + "/0/volFieldValue.dat";
        double localVelMag = readVelMagFileBFM(filePath + velMagFileName, latestTime);
        velRegionMags[i] = localVelMag;
    }


    //cout << "Force: " << totalForce[0] << ", " << totalForce[1] << ", " << totalForce[2] << endl;
    //cout << "Torque: " << totalTorque[0] << ", " << totalTorque[1] << ", " << totalTorque[2] << endl;

    return make_pair(totalForce, totalTorque);
}
