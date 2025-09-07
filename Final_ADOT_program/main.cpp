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

    vector<double> paramVals;
    paramVals.resize(paramRanges.columns.size());
    vector<string> paramNames;

    for(int j = 0; j < 10; j++){
        cout << j << endl;

    //Finds random values of parameters within bounds
    for(int i = 0; i < (int)paramVals.size(); i++){

        double min = paramRanges.columns[i].second[0];

        double max = paramRanges.columns[i].second[1];
        paramVals[i] = (double)rand()/RAND_MAX*(max-min)+min;

        paramNames.push_back(paramRanges.columns[i].first);

    }

    glm::dmat3 MOI;
    glm::dvec3 COM;
    double mass;

    vector<function<profile(vector<string>, vector<double>, double)>> profileFunctions = {fuselageProfile, wingProfile};
    function<void(vector<string>&, vector<double>&)> derivedParamsFunc = calcDerivedParams;

    aircraft MULEaircraft = aircraft(paramNames, derivedParamsFunc, profileFunctions);


    MULEaircraft.addPart("fuselage", 1000, extrudeFuselage, 0);
    MULEaircraft.addPart("rightWing", "fuselage", 1000, extrudeRightWing, 1);
    MULEaircraft.addPart("leftWing",  "fuselage", 1000, extrudeLeftWing, 1);

    try
    {
        MULEaircraft.calculateVals(paramVals, 50, 50, mass, COM, MOI);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';

    }
    

    

    }

    //MULEaircraft.plot(500, 500, paramVals, 20);


    //Quit SDL
    SDL_Quit();
	exit(0);

    return 0;
}