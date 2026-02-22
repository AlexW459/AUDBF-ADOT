#include "getForces.h"

using namespace std;


pair<glm::dvec3, glm::dvec3> getForces(string filePath, double& tailVelMag, glm::dvec3& tailForce, 
    glm::dvec3& tailTorque, double latestTime){


    //Regex expression for single line of file.
    //First group is time, the next three are components of pressure force, 
    //the next three are components of viscous force, the next three are
    //components of pressure torque, the next three are components of viscous torque
    regex forcesMatch("(\\d+\\.\\d+) +\\t\\(\\((-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\) \\((-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\)\\) \\(\\((-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\) \\((-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d) (-?\\d\\.\\d+e[\\+-]\\d\\d)\\)\\)");
    //0.002             	((8.8026221108e+00 -9.4824053925e-01 1.8044495585e+01) (-2.5786969248e-03 -1.1765188435e-03 9.9951061308e-02)) ((-2.9949115575e-01 -9.4618962994e+00 6.5449899611e-01) (2.7422164643e-04 -9.6754994108e-03 8.6147374731e-05))
    regex magUtMatch("(\\d\\.\\d+) +\\t?(\\-?\\d+\\.\\d+e[\\+\\-]\\d\\d)");
    //0.296             	3.3883516015e+01
    
    //Open files
    ifstream forcesFile, tailForcesFile, magUtFile;

    string forcesFileName = "postProcessing/aeroForces/0/forces.dat";
    forcesFile.open(filePath + forcesFileName);
    if(!forcesFile) throw runtime_error("Could not open file \"" + filePath + forcesFileName + "\" in file \"getForces.cpp\"");

    string tailForcesFileName = "postProcessing/tailAeroForces/0/forces.dat";
    tailForcesFile.open(filePath + tailForcesFileName);
    if(!tailForcesFile) throw runtime_error("Could not open file \"" + filePath + tailForcesFileName + "\" in file \"getForces.cpp\"");

    string magUtFileName = "postProcessing/tailUpstreamVelocity/0/volFieldValue.dat";
    magUtFile.open(filePath + magUtFileName);
    if(!magUtFile) throw runtime_error("Could not open file \"" + filePath + magUtFileName + "\" in file \"getForces.cpp\"");

    //Skip past headers
    string forcesLine, tailForcesLine, magUtLine;
    for(int i = 0; i < 5; i++){getline(forcesFile, forcesLine);}
    for(int i = 0; i < 5; i++){getline(tailForcesFile, tailForcesLine);}
    for(int i = 0; i < 6; i++){getline(magUtFile, magUtLine);}
    
    //Stores the last 3 values of each force
    vector<glm::dvec3> forceVals(3, glm::dvec3(0.0));
    vector<glm::dvec3> torqueVals(3, glm::dvec3(0.0));
    vector<glm::dvec3> tailForceVals(3, glm::dvec3(0.0));
    vector<glm::dvec3> tailTorqueVals(3, glm::dvec3(0.0));
    vector<double> magUtVals(3, 0.0);
    
    //Gets value of first line
    smatch forcesInfo, torqueInfo, tailForcesInfo, magUtInfo;
    regex_search(forcesLine, forcesInfo, forcesMatch);
    regex_search(tailForcesLine, tailForcesInfo, forcesMatch);
    regex_search(magUtLine, magUtInfo, magUtMatch);


    while(stod(forcesInfo.str(1)) < latestTime){
        getline(forcesFile, forcesLine);
        getline(tailForcesFile, tailForcesLine);
        getline(magUtFile, magUtLine);
        regex_search(forcesLine, forcesInfo, forcesMatch);
        regex_search(tailForcesLine, tailForcesInfo, forcesMatch);
        regex_search(magUtLine, magUtInfo, magUtMatch);


        //Adds matched values to queues
        glm::dvec3 pressureForce = glm::dvec3(stod(forcesInfo.str(2)), stod(forcesInfo.str(3)), stod(forcesInfo.str(4)) );
        glm::dvec3 viscousForce = glm::dvec3(stod(forcesInfo.str(5)), stod(forcesInfo.str(6)), stod(forcesInfo.str(7)) );
        glm::dvec3 pressureTorque = glm::dvec3(stod(forcesInfo.str(8)), stod(forcesInfo.str(9)), stod(forcesInfo.str(10)) );
        glm::dvec3 viscousTorque = glm::dvec3(stod(forcesInfo.str(11)), stod(forcesInfo.str(12)), stod(forcesInfo.str(13)) );
        glm::dvec3 tailPressureForce = glm::dvec3(stod(tailForcesInfo.str(2)), stod(tailForcesInfo.str(3)), stod(tailForcesInfo.str(4)) );
        glm::dvec3 tailViscousForce = glm::dvec3(stod(tailForcesInfo.str(5)), stod(tailForcesInfo.str(6)), stod(tailForcesInfo.str(7)) );
        glm::dvec3 tailPressureTorque = glm::dvec3(stod(tailForcesInfo.str(8)), stod(tailForcesInfo.str(9)), stod(tailForcesInfo.str(10)) );
        glm::dvec3 tailViscousTorque = glm::dvec3(stod(tailForcesInfo.str(11)), stod(tailForcesInfo.str(12)), stod(tailForcesInfo.str(13)) );

        forceVals.push_back(pressureForce + viscousForce);
        torqueVals.push_back(pressureTorque + viscousTorque);
        tailForceVals.push_back(tailPressureForce + tailViscousForce);
        tailTorqueVals.push_back(tailPressureTorque + tailViscousTorque);
        magUtVals.push_back(stod(magUtInfo.str(2)));
            
        //Pops first values from queues
        forceVals.erase(forceVals.begin());
        torqueVals.erase(torqueVals.begin());
        tailForceVals.erase(tailForceVals.begin());
        tailTorqueVals.erase(tailTorqueVals.begin());
        magUtVals.erase(magUtVals.begin());
    }

    forcesFile.close();
    tailForcesFile.close();
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
    //cout << "MagUt: " << tailVelMag << endl;

    return make_pair(totalForce, totalTorque);
}

