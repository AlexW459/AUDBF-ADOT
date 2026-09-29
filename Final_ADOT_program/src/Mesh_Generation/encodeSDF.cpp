#include "encodeSDF.h"

using namespace std;

void encodeSDF(vector<double> SDF, glm::dmat2x3 boundingBox, glm::dmat2x3 widerBox,
    glm::ivec3 SDFsize, glm::dvec3 COM, double boundingRadius, glm::ivec3 extraCells, 
    double CELL_GRADIENT, int pos, int test){

    string caseDir = "Aerodynamics_Simulation_IBM_Test_" + to_string(test);

    double grading = CELL_GRADIENT;

    //Writes correct bounding box (also clears force and velocity bounds)
    string boundScriptCall = string(projectRoot) + "/src/simScripts/IBM/updateBounds.sh " + 
        to_string(widerBox[0][0]) + " " + to_string(widerBox[0][1]) + " " +
        to_string(widerBox[0][2]) + " " + to_string(widerBox[1][0]) + " " + 
        to_string(widerBox[1][1]) + " " + to_string(widerBox[1][2]) + " " +
        to_string(boundingBox[0][0]) + " " + to_string(boundingBox[0][1]) + " " +
        to_string(boundingBox[0][2]) + " " + to_string(boundingBox[1][0]) + " " + 
        to_string(boundingBox[1][1]) + " " + to_string(boundingBox[1][2]) + " " +
        to_string(SDFsize[0]) + " " + to_string(SDFsize[1]) + " " + to_string(SDFsize[2]) + " " +
        to_string(extraCells[0]) + " " + to_string(extraCells[1]) + " " + to_string(extraCells[2]) + " " +
        to_string(grading) + " " + to_string(1.0/grading) + " " +
        to_string(COM[0]) + " " + to_string(COM[1]) + " " + to_string(COM[2]) + " " +
        to_string(boundingRadius) + " \"" + OPENFOAM_SOURCE + "\" " + " " + to_string(test);
        

    int failure = system(boundScriptCall.c_str());
    if(failure) throw runtime_error("Setting bounds failed in case " + to_string(test));

    
    //Adds SDF
    string solidDictFile = caseDir + "/solidDict";
    ofstream solidDictFileS;
    solidDictFileS.open(solidDictFile, ios::app);
    if(!solidDictFileS) throw runtime_error("Failed to open solidDict in case " + to_string(test));

    for(int i = 0; i < (int)SDF.size(); i++){
        solidDictFileS << to_string(SDF[i]) << " \n";
    }

    solidDictFileS << ");\n";
    solidDictFileS << "}\n";
    solidDictFileS << "}\n";

    solidDictFileS.close();

}