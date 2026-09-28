#include "readVelMagFileBFM.h"

using namespace std;

double readVelMagFileBFM(string filePath, double latestTime){
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