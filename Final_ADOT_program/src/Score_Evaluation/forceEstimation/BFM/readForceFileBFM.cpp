#include "readForceFileBFM.h"

using namespace std;

pair<glm::dvec3, glm::dvec3> readForceFileBFM(string filePath, double latestTime){
    ifstream forceFile;

    const string forceMatchString = string("(\\d+\\.\\d+) +\\t\\(\\((-?\\d\\.\\d+e[\\+-]\\d\\d)") + 
        string(" (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\) \\((-?\\d\\.\\d+e[\\+-]\\d\\d)") +
        string(" (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\)\\) \\(\\((-?\\d\\.\\d+e[\\+-]\\d\\d)")
        + string(" (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\) \\((-?\\d\\.\\d+e[\\+-]\\d\\d)")
        + string(" (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\)\\)");
    const regex forceMatch(forceMatchString);

    forceFile.open(filePath);
    if(!forceFile) throw runtime_error("Could not open file \"" + filePath + "\" in file \"getForces.cpp\"");

    //Skip past headers
    string forceLine;
    for(int i = 0; i < 5; i++){getline(forceFile, forceLine);}

    //Stores the last 3 values of force and torque
    vector<glm::dvec3> forceVals(3, glm::dvec3(0.0));
    vector<glm::dvec3> torqueVals(3, glm::dvec3(0.0));

    //Gets value of first line
    smatch forceInfo;
    regex_search(forceLine, forceInfo, forceMatch);

    while(stod(forceInfo.str(1)) < latestTime){
        getline(forceFile, forceLine);
        regex_search(forceLine, forceInfo, forceMatch);

        //Adds matched values to queues
        glm::dvec3 pressureForce = glm::dvec3(stod(forceInfo.str(2)), stod(forceInfo.str(3)), stod(forceInfo.str(4)) );
        glm::dvec3 viscousForce = glm::dvec3(stod(forceInfo.str(5)), stod(forceInfo.str(6)), stod(forceInfo.str(7)) );
        glm::dvec3 pressureTorque = glm::dvec3(stod(forceInfo.str(8)), stod(forceInfo.str(9)), stod(forceInfo.str(10)) );
        glm::dvec3 viscousTorque = glm::dvec3(stod(forceInfo.str(11)), stod(forceInfo.str(12)), stod(forceInfo.str(13)) );
        forceVals.push_back(pressureForce + viscousForce);
        torqueVals.push_back(pressureTorque + viscousTorque);

        //Pops first values from queues
        forceVals.erase(forceVals.begin());
        torqueVals.erase(torqueVals.begin());
    }

    forceFile.close();

    //Gets forces by averaging the last three values
    glm::dvec3 avgForce = (forceVals[0] + forceVals[1] + forceVals[2])/3.0;
    glm::dvec3 avgTorque = (torqueVals[0] + torqueVals[1] + torqueVals[2])/3.0;

    return make_pair(avgForce, avgTorque);
}