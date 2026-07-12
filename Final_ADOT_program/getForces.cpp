#include "getForces.h"

using namespace std;


std::pair<glm::dvec3, glm::dvec3> getForces(std::string filePath, int numForceRegions,
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
    pair<glm::dvec3, glm::dvec3> totalForces = readForceFile(filePath + mainForceFileName, latestTime);
    glm::dvec3 totalForce = totalForces.first;
    glm::dvec3 totalTorque = totalForces.second;

    //Gets force from each region
    regionForces.resize(numForceRegions);
    regionTorques.resize(numForceRegions);
    for(int i = 0; i < numForceRegions; i++){
        string forceFileName = "postProcessing/aeroForces_ " +  to_string(i) + "/0/forces.dat";
        pair<glm::dvec3, glm::dvec3> localForces = readForceFile(filePath + forceFileName, latestTime);
        totalForce += localForces.first;
        totalTorque += localForces.second;

        regionForces[i] = localForces.first;
        regionTorques[i] = localForces.second;
    }

    //Gets velocity magnitudes from each region
    for(int i = 0; i < numVelRegions; i++){
        string velMagFileName = "postProcessing/velocity_" + to_string(i) + "/0/volFieldValue.dat";
        double localVelMag = readVelMagFile(filePath + velMagFileName, latestTime);
        velRegionMags[i] = localVelMag;
    }


    //cout << "Force: " << totalForce[0] << ", " << totalForce[1] << ", " << totalForce[2] << endl;
    //cout << "Torque: " << totalTorque[0] << ", " << totalTorque[1] << ", " << totalTorque[2] << endl;

    return make_pair(totalForce, totalTorque);
}

pair<glm::dvec3, glm::dvec3> readForceFile(string filePath, double latestTime){
    ifstream forceFile;

    const string forceMatchString = string("(\\d+\\.\\d+) +\\t\\(\\((-?\\d\\.\\d+e[\\+-]\\d\\d)") + 
        string(" (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\) \\((-?\\d\\.\\d+e[\\+-]\\d\\d)") +
        string(" (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\)\\) \\(\\((-?\\d\\.\\d+e[\\+-]\\d\\d))")
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

double readVelMagFile(string filePath, double latestTime){
    ifstream velMagFile;
    
    const regex velMagMatch("(\\d\\.\\d+) +\\t?(\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d)");

    velMagFile.open(filePath);
    if(!velMagFile) throw runtime_error("Could not open file \"" + filePath + "\" in file \"getForces.cpp\"");

    //Skip past headers
    string velMagLine;
    for(int i = 0; i < 6; i++){getline(velMagFile, velMagLine);}

    //Stores last 3 values of velocity magnitude
    vector<double> velMagVals(3, 0.0);

    //Gets value of first line
    smatch velMagInfo;
    regex_search(velMagLine, velMagInfo, velMagMatch);

    while(stod(velMagInfo.str(1)) < latestTime){
        getline(velMagFile, velMagLine);
        regex_search(velMagLine, velMagInfo, velMagMatch);

        //Adds matched values to queues
        velMagVals.push_back(stod(velMagInfo.str(2)));

        //Pops first values from queues
        velMagVals.erase(velMagVals.begin());
    }

    velMagFile.close();

    //Gets velocity magnitude by averaging last three values
    double avgVelMag = (velMagVals[0] + velMagVals[1] + velMagVals[2])/3.0;

    return avgVelMag;
}