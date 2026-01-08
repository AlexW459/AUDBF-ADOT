#include "getForces.h"

using namespace std;


pair<glm::dvec3, glm::dvec3> getForces(string filePath, double& tailVelMag, glm::dvec3& tailForce, 
    glm::dvec3& tailTorque, double latestTime){


    //Regex expression for single line of file.
    //First group is time, the next three are components of pressure force, 
    //the next three are components of viscous force, the next three are
    //components of pressure torque, the next three are components of viscous torque
    regex forceMatch("(\\d+(?:\\.\\d+)?) +	\\(\\((\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d) (\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d) (\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d)\\) \\((\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d) (\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d) (\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d)\\)\\) \\(\\((\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d) (\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d) (\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d)\\) \\((\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d) (\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d) (\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d)\\)\\)");

    //Open file
    ifstream forceFile;
    string forceFileName = "postProcessing/aeroForces/0/forces.dat";
    forceFile.open(filePath + forceFileName);

    if(!forceFile) throw runtime_error("Could not open file \"" + forceFileName + "\" in file \"getForces.cpp\"");

    //Skip past header
    string forceLine;
    getline(forceFile, forceLine);
    getline(forceFile, forceLine);
    getline(forceFile, forceLine);
    getline(forceFile, forceLine);

    //Stores the last 3 values of each force
    vector<glm::dvec3> pressureForces(3, glm::dvec3(0.0));
    vector<glm::dvec3> viscousForces(3, glm::dvec3(0.0));
    vector<glm::dvec3> pressureTorques(3, glm::dvec3(0.0));
    vector<glm::dvec3> viscousTorques(3, glm::dvec3(0.0));
    

    smatch forceInfo;
    regex_search(forceLine, forceInfo, forceMatch);
    while(abs(stod(forceInfo.str(1)) - latestTime) > 1.0e-6){
        getline(forceFile, forceLine);
        regex_search(forceLine, forceInfo, forceMatch);

        if(forceInfo.str(1) == "" || latestTime - stod(forceInfo.str(1)) < -1.0e-6){
            throw runtime_error("Specified time " + to_string(latestTime) + "s is not present in " + filePath + forceFileName);
        }


        //Adds matched values to queues
        pressureForces.push_back(glm::dvec3(stod(forceInfo.str(2)), 
            stod(forceInfo.str(3)), stod(forceInfo.str(4))));
        viscousForces.push_back(glm::dvec3(stod(forceInfo.str(5)), 
            stod(forceInfo.str(6)), stod(forceInfo.str(7))));
        pressureTorques.push_back(glm::dvec3(stod(forceInfo.str(8)), 
            stod(forceInfo.str(9)), stod(forceInfo.str(10))));
        viscousTorques.push_back(glm::dvec3(stod(forceInfo.str(11)), 
            stod(forceInfo.str(12)), stod(forceInfo.str(13))));
            
        //Pops first values from queues
        pressureForces.erase(pressureForces.begin());
        viscousForces.erase(viscousForces.begin());
        pressureTorques.erase(pressureTorques.begin());
        viscousTorques.erase(viscousTorques.begin());
    }

    forceFile.close();

    //Gets forces by averaging the last three values
    glm::dvec3 pressureForce = (pressureForces[0] + pressureForces[1] + pressureForces[2])/3.0;
    glm::dvec3 viscousForce = (viscousForces[0] + viscousForces[1] + viscousForces[2])/3.0;
    glm::dvec3 pressureTorque = (pressureTorques[0] + pressureTorques[1] + pressureTorques[2])/3.0;
    glm::dvec3 viscousTorque = (viscousTorques[0] + viscousTorques[1] + viscousTorques[2])/3.0;


    //Open file
    ifstream tailForceFile;
    string tailForceFileName = "postProcessing/tailAeroForces/0/forces.dat";
    tailForceFile.open(filePath + tailForceFileName);

    if(!tailForceFile) throw runtime_error("Could not open file \"" + tailForceFileName + "\" in file \"getForces.cpp\"");

    //Skip past header
    string tailForceLine;
    getline(tailForceFile, tailForceLine);
    getline(tailForceFile, tailForceLine);
    getline(tailForceFile, tailForceLine);
    getline(tailForceFile, tailForceLine);

    //Stores the last 3 values of each force
    vector<glm::dvec3> tailPressureForces(3, glm::dvec3(0.0));
    vector<glm::dvec3> tailViscousForces(3, glm::dvec3(0.0));
    vector<glm::dvec3> tailPressureTorques(3, glm::dvec3(0.0));
    vector<glm::dvec3> tailViscousTorques(3, glm::dvec3(0.0));

    smatch tailForceInfo;
    regex_search(tailForceLine, tailForceInfo, forceMatch);
    while(abs(stod(tailForceInfo.str(1)) - latestTime) > 1.0e-6){

        getline(tailForceFile, tailForceLine);
        regex_search(tailForceLine, tailForceInfo, forceMatch);

        if(tailForceInfo.str(1) == "" || latestTime - stod(tailForceInfo.str(1)) < -1.0e-6){
            throw runtime_error("Specified time " + to_string(latestTime) + "s is not present in " + filePath + tailForceFileName);
        }


        //Adds matched values to queues
        tailPressureForces.push_back(glm::dvec3(stod(forceInfo.str(2)), 
            stod(forceInfo.str(3)), stod(forceInfo.str(4))));
        tailViscousForces.push_back(glm::dvec3(stod(forceInfo.str(5)), 
            stod(forceInfo.str(6)), stod(forceInfo.str(7))));
        tailPressureTorques.push_back(glm::dvec3(stod(forceInfo.str(8)), 
            stod(forceInfo.str(9)), stod(forceInfo.str(10))));
        tailViscousTorques.push_back(glm::dvec3(stod(forceInfo.str(11)), 
            stod(forceInfo.str(12)), stod(forceInfo.str(13))));
            
        //Pops first values from queues
        tailPressureForces.erase(tailPressureForces.begin());
        tailViscousForces.erase(tailViscousForces.begin());
        tailPressureTorques.erase(tailPressureTorques.begin());
        tailViscousTorques.erase(tailViscousTorques.begin());
    }

    tailForceFile.close();


    //Gets forces by averaging the last three values
    glm::dvec3 tailPressureForce = (tailPressureForces[0] + tailPressureForces[1] + tailPressureForces[2])/3.0;
    glm::dvec3 tailViscousForce = (tailViscousForces[0] + tailViscousForces[1] + tailViscousForces[2])/3.0;
    glm::dvec3 tailPressureTorque = (tailPressureTorques[0] + tailPressureTorques[1] + tailPressureTorques[2])/3.0;
    glm::dvec3 tailViscousTorque = (tailViscousTorques[0] + tailViscousTorques[1] + tailViscousTorques[2])/3.0;

    tailForce = tailPressureForce + tailViscousForce;
    tailTorque = tailPressureTorque + tailViscousTorque;

    glm::dvec3 totalForce = pressureForce + viscousForce;
    glm::dvec3 totalTorque = pressureTorque + viscousTorque;

    //cout << "Force: " << totalForce[0] << ", " << totalForce[1] << ", " << totalForce[2] << endl;
    //cout << "Torque: " << totalTorque[0] << ", " << totalTorque[1] << ", " << totalTorque[2] << endl;

    //Get velocity magnitude in front of tail
    ifstream magUtFile;
    string magUtFileName = "postProcessing/tailUpstreamVelocity/0/volFieldValue.dat";
    magUtFile.open(filePath + magUtFileName);

    if(!magUtFile) throw runtime_error("Could not open file \"" + magUtFileName + "\" in file \"getForces.cpp\"");

    //Skip past header
    string magUtLine;
    getline(magUtFile, magUtLine);
    getline(magUtFile, magUtLine);
    getline(magUtFile, magUtLine);
    getline(magUtFile, magUtLine);
    getline(magUtFile, magUtLine);

    regex magUtMatch("(\\d+(?:\\.\\d+)?) +	(\\d+\\.\\d+e[\\+\\-]\\d\\d)");

    smatch magUtInfo;
    regex_search(magUtLine, magUtInfo, magUtMatch);
    while(abs(stod(magUtInfo.str(1)) - latestTime) > 1.0e-6){
        getline(magUtFile, magUtLine);
        regex_search(magUtLine, magUtInfo, magUtMatch);

        if(magUtInfo.str(1) == "" || latestTime - stod(magUtInfo.str(1)) < -1.0e-6){
            throw runtime_error("Specified time " + to_string(latestTime) + "s is not present in " + filePath + magUtFileName);
        }
    }

    magUtFile.close();

    tailVelMag = stod(magUtInfo.str(2));

    return make_pair(totalForce, totalTorque);
}

