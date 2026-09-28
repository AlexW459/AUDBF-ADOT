#include "Cube.h"

using namespace std;

extern "C" testModel constructModel(){

    // Creates vector of profile functions
    vector<function<profile(vector<string>, vector<double>, double)>> profileFunctions = 
        {cubeProfile};
    
    // Gets maximum and minimun values of each parameter
    dataTable paramTable = readCSV("Cube/paramRanges.csv");
    int nParams = paramTable.rows.size();

    vector<dataTable> discreteTables = {};

    // Gets names and ranges of parameters
    vector<string> paramNames(nParams);
    vector<glm::dvec2> paramRanges(nParams);
    for(int i = 0; i < nParams; i++){
        paramNames[i] = paramTable.rows[i].first;
        paramRanges[i][0] = paramTable.rows[i].second[0];
        paramRanges[i][1] = paramTable.rows[i].second[1];
    }

    // Describes positions at which simulations will be conducted
    double testVelocity = 10.0;
    vector<vector<double>> positionValues = {{-testVelocity, 0.0, 0.0, 0.0, 0.0, -9.8}};

    testModel cubeModel(paramNames, paramRanges, discreteTables, calcDerivedParams, profileFunctions, 
        positionValues, rateDesign);

    cubeModel.addPart("Cube", 1.0, extrudeCube, 0);

    //cubeModel.plot(500, 500, paramVals, discreteVals, 50.0);

    return cubeModel;
}

void calcDerivedParams(vector<string>& paramNames, vector<double>& paramVals, 
    const vector<dataTable>& discreteTables, vector<glm::dmat2x3>& velocityRegions){
}

profile cubeProfile(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double sideLength = getParam("sideLength", paramVals, paramNames);

    vector<glm::dvec2> points = {glm::dvec2(-0.5*sideLength, -0.5*sideLength), 
        glm::dvec2(-0.5*sideLength, 0.5*sideLength), 
        glm::dvec2(0.5*sideLength, 0.5*sideLength), 
        glm::dvec2(0.5*sideLength, -0.5*sideLength)};

    //vector<glm::dvec2> points = {glm::dvec2(-0.5, -0.5), glm::dvec2(0.5, -0.5), 
    //    glm::dvec2(0.5, 0.5), glm::dvec2(-0.5, 0.5)};

    profile outputProfile(points);

    return outputProfile;
}

extrusionData extrudeCube(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double sideLength = getParam("sideLength", paramVals, paramNames);

    vector<double> zSampleVals = {-0.5*sideLength, 0.5*sideLength};
    vector<glm::dvec2> posVals = {glm::dvec2(0.0), glm::dvec2(0.0)};
    vector<glm::dvec2> scaleVals = {glm::dvec2(1.0), glm::dvec2(1.0)};

    glm::dvec3 translation(0.0);
    glm::dquat rotation(glm::dvec3(M_PI/4.0, 0.0, 0.0));
    glm::dvec3 pivotPoint(0.0);

    extrusionData cubeExtrusion(zSampleVals, posVals, scaleVals, rotation, translation, pivotPoint);

    return cubeExtrusion;

}

double rateDesign(vector<string> fullParamNames, vector<double> fullParamVals, vector<vector<double>> positionVariables,
    double mass, vector<glm::dvec3> COMs, vector<glm::dmat3> MOIs, vector<glm::dvec3> totalForces, 
    vector<glm::dvec3> totalTorques, vector<vector<glm::dvec3>> regionForces,
    vector<vector<glm::dvec3>> regionTorques, vector<vector<double>> regionVelMags,
    vector<vector<glm::dvec3>> POIs, vector<glm::dvec3> partDirections){

    //cout << totalForces[0][2] << endl;

    return abs(totalForces[0][2]);
}