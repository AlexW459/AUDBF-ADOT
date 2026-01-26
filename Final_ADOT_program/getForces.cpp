#include "getForces.h"

using namespace std;


pair<glm::dvec3, glm::dvec3> getForces(string filePath, double& tailVelMag, glm::dvec3& tailForce, 
    glm::dvec3& tailTorque, double latestTime){


    //Regex expression for single line of file.
    //First group is time, the next three are components of pressure force, 
    //the next three are components of viscous force, the next three are
    //components of pressure torque, the next three are components of viscous torque
    regex forceMatch("(\\d\\.\\d+) +\\t?(\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d) (\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d) (\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d)");
    //0.004              -8.2328726659e-01 -1.5527631999e-02 1.5465345119e+01 -8.0404813389e-01 -1.6498713040e-02 1.5409775875e+01 -1.9239132703e-02 9.7108104161e-04 5.5569244608e-02
    regex magUtMatch("(\\d\\.\\d+) +\\t?(\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d)");
    //0.296             	3.3883516015e+01
    
    //Open files
    ifstream forceFile, torqueFile, tailForceFile, tailTorqueFile, magUtFile;

    string forceFileName = "postProcessing/aeroForces/0/force.dat";
    forceFile.open(filePath + forceFileName);
    if(!forceFile) throw runtime_error("Could not open file \"" + forceFileName + "\" in file \"getForces.cpp\"");

    string torqueFileName = "postProcessing/aeroForces/0/moment.dat";
    torqueFile.open(filePath + torqueFileName);
    if(!torqueFile) throw runtime_error("Could not open file \"" + torqueFileName + "\" in file \"getForces.cpp\"");

    string tailForceFileName = "postProcessing/tailAeroForces/0/force.dat";
    tailForceFile.open(filePath + tailForceFileName);
    if(!tailForceFile) throw runtime_error("Could not open file \"" + tailForceFileName + "\" in file \"getForces.cpp\"");

    string tailTorqueFileName = "postProcessing/tailAeroForces/0/force.dat";
    tailTorqueFile.open(filePath + tailTorqueFileName);
    if(!tailTorqueFile) throw runtime_error("Could not open file \"" + tailTorqueFileName + "\" in file \"getForces.cpp\"");

    string magUtFileName = "postProcessing/tailUpstreamVelocity/0/volFieldValue.dat";
    magUtFile.open(filePath + magUtFileName);
    if(!magUtFile) throw runtime_error("Could not open file \"" + magUtFileName + "\" in file \"getForces.cpp\"");

    //Skip past headers
    string forceLine, torqueLine, tailForceLine, tailTorqueLine, magUtLine;
    for(int i = 0; i < 5; i++){getline(forceFile, forceLine);}
    for(int i = 0; i < 5; i++){getline(torqueFile, torqueLine);}
    for(int i = 0; i < 5; i++){getline(tailForceFile, tailForceLine);}
    for(int i = 0; i < 5; i++){getline(tailTorqueFile, tailTorqueLine);}
    for(int i = 0; i < 5; i++){getline(magUtFile, magUtLine);}
    
    //Stores the last 3 values of each force
    vector<glm::dvec3> forceVals(3, glm::dvec3(0.0));
    vector<glm::dvec3> torqueVals(3, glm::dvec3(0.0));
    vector<glm::dvec3> tailForceVals(3, glm::dvec3(0.0));
    vector<glm::dvec3> tailTorqueVals(3, glm::dvec3(0.0));
    vector<double> magUtVals(3, 0.0);
    
    //Gets value of first line
    smatch forceInfo, torqueInfo, tailForceInfo, tailTorqueInfo, magUtInfo;
    regex_search(forceLine, forceInfo, forceMatch);
    regex_search(torqueLine, torqueInfo, forceMatch);
    regex_search(tailForceLine, tailForceInfo, forceMatch);
    regex_search(tailTorqueLine, tailTorqueInfo, forceMatch);
    regex_search(magUtLine, magUtInfo, magUtMatch);

    while(stod(forceInfo.str(1)) < latestTime){
        getline(forceFile, forceLine);
        getline(torqueFile, torqueLine);
        getline(tailForceFile, tailForceLine);
        getline(tailTorqueFile, tailTorqueLine);
        getline(magUtFile, magUtLine);
        regex_search(forceLine, forceInfo, forceMatch);
        regex_search(torqueLine, torqueInfo, forceMatch);
        regex_search(tailForceLine, tailForceInfo, forceMatch);
        regex_search(tailTorqueLine, tailTorqueInfo, forceMatch);
        regex_search(magUtLine, magUtInfo, magUtMatch);

        //Adds matched values to queues
        forceVals.push_back(glm::dvec3(stod(forceInfo.str(2)), 
            stod(forceInfo.str(3)), stod(forceInfo.str(4))));
        torqueVals.push_back(glm::dvec3(stod(torqueInfo.str(2)), 
            stod(torqueInfo.str(3)), stod(torqueInfo.str(4))));
        tailForceVals.push_back(glm::dvec3(stod(tailForceInfo.str(2)), 
            stod(tailForceInfo.str(3)), stod(tailForceInfo.str(4))));
        tailTorqueVals.push_back(glm::dvec3(stod(tailTorqueInfo.str(2)), 
            stod(tailTorqueInfo.str(3)), stod(tailTorqueInfo.str(4))));
        magUtVals.push_back(stod(magUtInfo.str(2)));
            
        //Pops first values from queues
        forceVals.erase(forceVals.begin());
        torqueVals.erase(torqueVals.begin());
        tailForceVals.erase(tailForceVals.begin());
        tailTorqueVals.erase(tailTorqueVals.begin());
        magUtVals.erase(magUtVals.begin());
    }

    forceFile.close();
    torqueFile.close();
    tailForceFile.close();
    tailTorqueFile.close();
    magUtFile.close();

    //Gets forces by averaging the last three values
    glm::dvec3 avgForce = (forceVals[0] + forceVals[1] + forceVals[2])/3.0;
    glm::dvec3 avgTorque = (torqueVals[0] + torqueVals[1] + torqueVals[2])/3.0;
    glm::dvec3 avgTailForce = (tailForceVals[0] + tailForceVals[1] + tailForceVals[2])/3.0;
    glm::dvec3 avgTailTorque = (tailTorqueVals[0] + tailTorqueVals[1] + tailTorqueVals[2])/3.0;
    double avgMagUt = (magUtVals[0] + magUtVals[1] + magUtVals[2])/3.0;

    //Total torque excludes torque on tail
    glm::dvec3 totalForce = avgForce;
    glm::dvec3 totalTorque = avgTorque;
    tailForce = avgTailForce;
    tailTorque = avgTailTorque;

    tailVelMag = avgMagUt;

    //cout << "Force: " << totalForce[0] << ", " << totalForce[1] << ", " << totalForce[2] << endl;
    //cout << "Torque: " << totalTorque[0] << ", " << totalTorque[1] << ", " << totalTorque[2] << endl;
    //cout << "MagUt: " << tailVelMag s<< endl;

    return make_pair(totalForce, totalTorque);
}

