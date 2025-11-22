#include "extrusionGen.h"


//Generates volumetric mesh. returns the number of tetrahedrons in the mesh
int generateExtrusion(const profile& partProfile, const extrusionData& extrusion, 
    vector<char>& adjMatrix, vector<glm::dvec3>& points, glm::dmat2x3& boundingBox){


    int numProfiles = extrusion.zSampleVals.size();
    int profileSize = partProfile.vertexCoords.size();
    int outerSize = profileSize;


    if(partProfile.inset){
        profileSize *= 2;
    }

    points.resize(numProfiles*profileSize);

    vector<glm::dvec2> newPoints;
    newPoints.resize(profileSize);


    //Finds positions of profiles
    for(int i = 0; i < numProfiles; i++){
        float zPos = extrusion.zSampleVals[i];
        int profileBegin = i*profileSize;

        glm::dvec3 profileShift = glm::dvec3(extrusion.posVals[i], 0);

        //cout << profileShift[0] << ", " << profileShift[1] << endl;
        glm::dvec2 profileScale = extrusion.scaleVals[i];
        for(int j = 0; j < outerSize; j++){
            //Shifts and scales points
            glm::dvec3 newPoint = glm::dvec3(partProfile.vertexCoords[j] * profileScale, zPos) + profileShift;
            newPoints[j] = newPoint;
            points[profileBegin + j] = newPoint;

        }


        //Adds inset points
        vector<glm::dvec2> insetPoints = partProfile.generateInset(newPoints);
        //Loop does not run if there is no inset
        for(int j = 0; j < profileSize-outerSize; j++){
            glm::dvec3 newPoint = glm::dvec3(insetPoints[j] * profileScale, zPos) + profileShift;
            points[profileBegin + outerSize + j] = newPoint;
        }

    }

    //Gets maximum distance from points to axis of rotation and the endpoints of the smallest cylinder
    //that encloses all of the points

    //if(!extrusion.isControl){

        //Initialises bounding box

        //Checks for reverse order extrusions
        if(extrusion.zSampleVals[0] < extrusion.zSampleVals[1]){
            boundingBox[0][2] = extrusion.zSampleVals[0];
            boundingBox[1][2] = extrusion.zSampleVals.back();
        }else{
            boundingBox[0][2] = extrusion.zSampleVals.back();
            boundingBox[1][2] = extrusion.zSampleVals[0];
        }


        boundingBox[0][0] = 10.0;
        boundingBox[0][1] = 10.0;
        boundingBox[1][0] = -10.0;
        boundingBox[1][1] = -10.0;

        //Only checks outer points
        for(int i = 0; i < numProfiles; i++){
            for(int j = 0; j < outerSize; j++){
                int index = i*profileSize + j;

                //cout << "point: " << points[index][0] << ", " << points[index][1] << ", " << points[index][2] << endl;
                
                //Updates bounding box with new minimums and maximums
                boundingBox[0][0] += (points[index][0] - boundingBox[0][0]) *  (points[index][0] < boundingBox[0][0]);
                boundingBox[0][1] += (points[index][1] - boundingBox[0][1]) *  (points[index][1] < boundingBox[0][1]);
                
                boundingBox[1][0] += (points[index][0] - boundingBox[1][0]) *  (points[index][0] > boundingBox[1][0]);
                boundingBox[1][1] += (points[index][1] - boundingBox[1][1]) *  (points[index][1] > boundingBox[1][1]);


            }
        }

        

    int adjSize = numProfiles*profileSize;
    vector<char> profileAdjMatrix = partProfile.adjacencyMatrix;

    //Initialises adjacency matrix
    adjMatrix.resize(adjSize*adjSize);
    for(int i = 0; i < adjSize; i++){
        //Fills with zeros initially
        for(int j = 0; j < adjSize; j++){
            adjMatrix[i*adjSize + j] = 0;
        }
    }

    //Adds profile adjacency matrix to adjacency matrix
    for(int p = 0; p < numProfiles; p++){
        int startI = p*profileSize;

        for(int i = 0; i < profileSize; i++){
            for(int j = i; j < profileSize; j ++){

                int ind1 = startI + i;
                int ind2 = startI + j;
                adjMatrix[ind1*adjSize + ind2] = profileAdjMatrix[i*profileSize + j];
                adjMatrix[ind2*adjSize + ind1] = profileAdjMatrix[i*profileSize + j];

            }
        }        
    }

    //Adds identity matrices to matrix
    for(int i = profileSize; i < profileSize*numProfiles; i++){
        adjMatrix[(i-profileSize)*adjSize + i] = 1;
        adjMatrix[i*adjSize + (i-profileSize)] = 1;
    }

    //Splits rectangles into triangles
    for(int p = 0; p < numProfiles-1; p++){
        for(int i = 0; i < profileSize; i++){
            for(int j = i+1; j < profileSize; j ++){
                int ind1 = p*profileSize + i;
                int ind2 = (p + 1)*profileSize + j;
                adjMatrix[ind1*adjSize + ind2] = profileAdjMatrix[i*profileSize + j];
                adjMatrix[ind2*adjSize + ind1] = profileAdjMatrix[i*profileSize + j];
            }
        }
    }

    //Find number of tetrahedrons from adjacency matrix
    int numTetras = 3*partProfile.numTriangles*(numProfiles-1);


    return numTetras;
}

double getParam(string param, const vector<double>& paramVals, const vector<string>& paramNames){

    int index = distance(paramNames.begin(), find(paramNames.begin(), 
        paramNames.end(), param));


    if(index == (int)paramNames.size()){
        throw std::runtime_error("Could not find variable \"" + param + "\" in variable list");
        return 0.0;
    }else{
        return paramVals[index];
    }

}