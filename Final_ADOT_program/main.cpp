#include "main.h"


using namespace std;

int main(){
    //Initialise rand()
    srand (time(0));
    //Initialise SDL
    SDL_Init(SDL_INIT_EVERYTHING);

    string planeModel = "MULEplaneModel";

    //Gets maximum and minimun values of each parameter
    dataTable paramRanges = readCSV(planeModel + "/paramRanges.csv");
    dataTable motorTable = readCSV(planeModel + "/motorSelection.csv");
    dataTable batteryTable = readCSV(planeModel + "/batterySelection.csv");

    vector<double> paramVals;
    paramVals.resize(paramRanges.rows.size());
    vector<string> paramNames;
    paramNames.resize(paramRanges.rows.size());

    vector<dataTable> discreteTables = {motorTable, batteryTable};
    vector<int> discreteVals;
    discreteVals.resize(discreteTables.size());
    

    //Finds random values of parameters within bounds
    for(int i = 0; i < (int)paramVals.size(); i++){

        double min = paramRanges.rows[i].second[0];

        double max = paramRanges.rows[i].second[1];

        paramVals[i] = (double)rand()/RAND_MAX*(max-min)+min;

        paramNames[i] = paramRanges.rows[i].first;

    }


    //Finds random values of discrete parameters
    for(int i = 0; i < (int)discreteTables.size(); i++){

        int numChoices = discreteTables[i].rows.size();

        discreteVals[i] = rand() % numChoices;
    }

    vector<glm::dmat3> MOIs;
    vector<glm::dvec3> COMs;
    double mass;


    vector<function<profile(vector<string>, vector<double>, double)>> profileFunctions = 
        {fuselageProfile, wingProfile, motorPodProfile, elevatorProfile};
    

    aircraft MULEaircraft = aircraft(paramNames, discreteTables, calcDerivedParams, profileFunctions);


    MULEaircraft.addPart("fuselage", 1000, extrudeFuselage, 0);
    MULEaircraft.addPart("rightWing", "fuselage", false, 1000, extrudeRightWing, 1);
    MULEaircraft.addPart("leftWing", "fuselage", false, 1000, extrudeLeftWing, 1);
    //MULEaircraft.addPart("elevatorRight", "rightWing", true, 1000, extrudeRightElevator, 3);


    MULEaircraft.calculateVals(paramVals, discreteVals, 50.0, 100.0, mass, COMs, MOIs);


    MULEaircraft.plot(500, 500, paramVals, discreteVals, 50.0);

    //Quit SDL
    SDL_Quit();
	exit(0);

    return 0;
}