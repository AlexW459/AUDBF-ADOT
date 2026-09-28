#include "generateOFmesh.h"

using namespace std;

void generateOFmesh(vector<double> posVals, glm::dmat2x3 boundingBox, glm::dvec3 pointInMesh,
    vector<glm::dmat2x3> forceRegions, vector<glm::dmat2x3> velRegions, int position, int test){

    glm::dvec3 flowVelocity(posVals[0], posVals[1], posVals[2]);
    int failure;

    glm::dmat2x3 widerBoundingBox = boundingBox;
    widerBoundingBox[1] -= 0.6*glm::normalize(flowVelocity);
    widerBoundingBox[0] += 0.6*glm::normalize(flowVelocity);
    //cout << "Setting bounds on rank " << procRank << endl;

    //Writes correct bounding box (also clears force and velocity bounds)
    string boundScriptCall = string(projectRoot) + "/src/simScripts/BFM/updateBounds.sh " + 
        to_string(widerBoundingBox[0][0]) + " " + to_string(widerBoundingBox[0][1]) + " " +
        to_string(widerBoundingBox[0][2]) + " " + to_string(widerBoundingBox[1][0]) + " " + 
        to_string(widerBoundingBox[1][1]) + " " + to_string(widerBoundingBox[1][2]) + " " +
        to_string(boundingBox[0][0]) + " " + to_string(boundingBox[0][1]) + " " + 
        to_string(boundingBox[0][2]) + " " + to_string(boundingBox[1][0]) + " " + 
        to_string(boundingBox[1][1]) + " " + to_string(boundingBox[1][2]) + " " +
        to_string(pointInMesh[0]) + " " + to_string(pointInMesh[1]) + " " + to_string(pointInMesh[2]) + " " +
        to_string(position) + " " + to_string(test);

    failure = system(boundScriptCall.c_str());
    if(failure) throw runtime_error("Setting bounds failed in case " + to_string(position));


    // Adds force bounds
    /*int numForceRegions = forceRegions.size();
    for (int f = 0; f < numForceRegions; f++){
        glm::dmat2x3 forceRegion = forceRegions[f];
        string forceRegionBounds = to_string(forceRegion[0][0]) + " " + to_string(forceRegion[0][1]) + " " +
        to_string(forceRegion[0][2]) + " " + to_string(forceRegion[1][0]) + " " + to_string(forceRegion[1][1]) +
        " " + to_string(forceRegion[1][2]);
        string addForceScriptCall = string(projectRoot) + "/src/simScripts/BFM/addForceRegion.sh" + " " + to_string(position) + " " + 
            to_string(f) + " " + forceRegionBounds;
        failure = system(addForceScriptCall.c_str());
        if(failure) throw runtime_error("Adding force regions failed in case " + to_string(position));
    }

    // Adds a final parantheses to file
    string paranthesesCall = "cat >> Aerodynamics_Simulation_BFM_" + to_string(position) + "/system/createPatchDict <<EOF\n}\nEOF";
    failure = system(paranthesesCall.c_str());
    if(failure) throw runtime_error("Adding force regions failed in case " + to_string(position));

    // Adds velocity bounds
    int numVelRegions = velRegions.size();
    for (int v = 0; v < numVelRegions; v++){
        glm::dmat2x3 velRegion = velRegions[v];
        string velRegionBounds = to_string(velRegion[0][0]) + " " + to_string(velRegion[0][1]) + " " +
        to_string(velRegion[0][2]) + " " + to_string(velRegion[1][0]) + " " + to_string(velRegion[1][1]) +
        " " + to_string(velRegion[1][2]);
        string addVelScriptCall = string(projectRoot) + "/src/simScripts/BFM/addVelRegion.sh" + " " + to_string(position) 
            + " " + to_string(v) + " " + velRegionBounds;
        failure = system(addVelScriptCall.c_str());
        if(failure) throw runtime_error("Adding velocity region " + to_string(v) + " failed in case " + to_string(position));
    }*/
    

    cout << "Meshing on position " << position << " and test " << test << endl;

    string meshScriptCall = string(projectRoot) + "/src/simScripts/BFM/meshObj.sh " + to_string(position) + " " + 
        " \"" + OPENFOAM_SOURCE + "\" ";
    failure = system(meshScriptCall.c_str());
    if(failure) throw runtime_error("Meshing failed in case " + to_string(position));

    cout << "Completed meshing on position " << position << endl;
}