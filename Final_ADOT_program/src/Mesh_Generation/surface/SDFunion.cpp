#include "SDFunion.h"

using namespace std;

void SDFunion(vector<double>& initialSDF, const vector<double> secondarySDF){

    int SDFsize = initialSDF.size();
    if(initialSDF.size() != secondarySDF.size()) throw runtime_error("SDF sizes do not match in SDFunion()");

    //Find minimum of part SDF and total SDF for points in bounding box
    for(int i = 0; i < SDFsize; i++){
        //Vectorisable min function
        //initialSDF[i] = initialSDF[i] + (secondarySDF[i] - initialSDF[i]) * (secondarySDF[i] < initialSDF[i]);
        initialSDF[i] = min(initialSDF[i], secondarySDF[i]);
    }

}
