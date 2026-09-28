#include "getForcesIBM.h"

using namespace std;

pair<glm::dvec3, glm::dvec3> getForcesIBM(string filePath, double latestTime){

    string forceFileName = "cloud.out";
    string valMatch = "-?\\d.\\d{6}e[\\+-]\\d\\d";
    const string forceMatchString = "(" + valMatch + ") " + valMatch + " " + valMatch + " " +
        valMatch + " " + valMatch + " " + valMatch + " " + valMatch + " (" + valMatch + ") (" +
        valMatch + ") (" + valMatch + ") (" + valMatch + ") (" + valMatch + ") (" + valMatch + ")";
    const regex forceMatch(forceMatchString);

    ifstream forceFile;
    forceFile.open(filePath + forceFileName);
    if(!forceFile) throw runtime_error("Failed to open forces file in " + filePath);

    //Stores the last 3 values of force and torque
    vector<glm::dvec3> forceVals(3, glm::dvec3(0.0));
    vector<glm::dvec3> torqueVals(3, glm::dvec3(0.0));

    //Gets value of first line
    smatch forceInfo;
    string forceLine;
    getline(forceFile, forceLine);
    regex_search(forceLine, forceInfo, forceMatch);

    while(stod(forceInfo.str(1)) < latestTime){
        getline(forceFile, forceLine);
        regex_search(forceLine, forceInfo, forceMatch);

        //Adds matched values to queues
        glm::dvec3 force = glm::dvec3(stod(forceInfo.str(2)), stod(forceInfo.str(3)), stod(forceInfo.str(4)) );
        glm::dvec3 torque = glm::dvec3(stod(forceInfo.str(5)), stod(forceInfo.str(6)), stod(forceInfo.str(7)) );
        forceVals.push_back(force);
        torqueVals.push_back(torque);

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