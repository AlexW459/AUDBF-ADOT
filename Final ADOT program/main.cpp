#include "main.h"

using namespace std;

int main(){
    //Initialise rand()
    srand (time(0));
    //Initialise SDL
    SDL_Init(SDL_INIT_EVERYTHING);

    string planeModel = "MULEplaneModel";

    dataTable paramRanges = readCSV(planeModel + "/paramRanges.csv");

    vector<double> paramVals;
    paramVals.resize(paramRanges.columns.size());
    vector<string> paramNames;

    for(int i = 0; i < (int)paramVals.size(); i++){

        double min = paramRanges.columns[i].second[0];

        double max = paramRanges.columns[i].second[1];
        paramVals[i] = rand()/RAND_MAX*(max-min)+min;

        paramNames.push_back(paramRanges.columns[i].first);
    }

    //glm::mat3 MOI;
    //glm::vec3 COM;
    //float mass;

    vector<function<profile*(vector<string>, vector<double>, double)>> profileFunctions = {fuselageProfile};
    function<void(vector<string>&, vector<double>&)> derivedParamsFunc = calcDerivedParams;

    aircraft MULEaircraft = aircraft(paramNames, derivedParamsFunc, profileFunctions);

    MULEaircraft.addPart("Fuselage", 1000, extrudeFuselage, 0);

    //MULEaircraft.calculateVals(paramVals, 20, 20, mass, COM, MOI);

    MULEaircraft.plot(500, 500, paramVals, 20);


    //Quit SDL
    SDL_Quit();
	exit(0);

    return 0;
}