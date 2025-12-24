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

    vector<double> paramVals(paramRanges.rows.size());
    vector<string> paramNames(paramRanges.rows.size());

    vector<dataTable> discreteTables = {motorTable, batteryTable};
    vector<int> discreteVals(discreteTables.size());
    

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


    vector<function<profile(vector<string>, vector<double>, double)>> profileFunctions = 
        {fuselageProfile, wingProfile, motorPodProfile, empennageBoomProfile, 
            horizontalStabiliserProfile, elevatorProfile};
    

    aircraft MULEaircraft = aircraft(paramNames, discreteTables, calcDerivedParams, profileFunctions, 0.0005);


    MULEaircraft.addPart("fuselage", 1000, extrudeFuselage, 0);
    MULEaircraft.addPart("rightWing", "fuselage", 1000, extrudeRightWing, 1);
    MULEaircraft.addPart("leftWing", "fuselage", 1000, extrudeLeftWing, 1);
    MULEaircraft.addPart("motorPodRight", "rightWing", 1000, extrudeRightMotorPod, 2);
    MULEaircraft.addPart("motorPodLeft", "leftWing", 1000, extrudeLeftMotorPod, 2);
    MULEaircraft.addPart("empennageBoomRight", "motorPodRight", 1000, extrudeEmpennageBoom, 3);
    MULEaircraft.addPart("empennageBoomLeft", "motorPodLeft", 1000, extrudeEmpennageBoom, 3);
    MULEaircraft.addPart("horizontalStabiliser", "empennageBoomRight", 1000, extrudeHorizontalStabiliser, 4);
    MULEaircraft.addPart("elevator", "horizontalStabiliser", 1000, extrudeElevator, 5);

    //Rate aircraft performance
    array<double, 3> aircraftConfig;
    double score = MULEaircraft.calculateScore(paramVals, discreteVals, rateDesign, aircraftConfig, 75.0, 150.0);


    //MULEaircraft.plot(500, 500, paramVals, discreteVals, 50.0);

    //Quit SDL
    SDL_Quit();
    
    return 0;
}