#include "extrusionGen.h"



//Generates volumetric mesh. returns the number of tetrahedrons in the mesh
int generateExtrusion(const profile& partProfile, const extrusionData& extrusion, 
    vector<char>& adjMatrix, vector<glm::vec3>& points, glm::mat2x3& boundingBox){

    int numProfiles = extrusion.zSampleVals.size();
    int profileSize = partProfile.vertexCoords.size();
    int outerSize = profileSize;


    if(partProfile.inset){
        profileSize *= 2;
    }

    points.resize(numProfiles*profileSize);

    vector<glm::vec2> newPoints;
    newPoints.resize(profileSize);


    float extrusionBegin = *min_element(extrusion.zSampleVals.begin(), extrusion.zSampleVals.end());

    float extrusionEnd = *max_element(extrusion.zSampleVals.begin(), extrusion.zSampleVals.end());

    //Initialises bounding box
    boundingBox[0][2] = extrusionBegin;
    boundingBox[1][2] = extrusionEnd;

    boundingBox[0][0] = 1e6;
    boundingBox[0][1] = 1e6;
    boundingBox[1][0] = -1e6;
    boundingBox[1][1] = -1e6;


    //Finds positions of profiles
    for(int i = 0; i < numProfiles; i++){
        float zPos = extrusion.zSampleVals[i];
        int profileBegin = i*profileSize;

        float profileXShift = extrusion.xPosVals[i];
        float profileYShift = extrusion.yPosVals[i];
        float profileScale = extrusion.scaleVals[i];
        for(int j = 0; j < outerSize; j++){
            glm::vec3 newPoint(partProfile.vertexCoords[j] * profileScale, zPos);
            newPoint[1] += profileYShift;
            newPoint[0] += profileXShift;
            newPoints[j] = newPoint;
            points[profileBegin + j] = newPoint;

            //Updates bounding box with new minimums and maximums
            boundingBox[0][0] += (newPoint[0] - boundingBox[0][0]) *  (newPoint[0] < boundingBox[0][0]);
            boundingBox[0][1] += (newPoint[1] - boundingBox[0][1]) *  (newPoint[1] < boundingBox[0][1]);
            boundingBox[1][0] += (newPoint[0] - boundingBox[1][0]) *  (newPoint[0] > boundingBox[1][0]);
            boundingBox[1][1] += (newPoint[1] - boundingBox[1][1]) *  (newPoint[1] > boundingBox[1][1]);

        }


        //Adds inset points
        vector<glm::vec2> insetPoints = partProfile.generateInset(newPoints);
        //Loop does not run if there is no inset
        for(int j = 0; j < profileSize-outerSize; j++){
            glm::vec3 newPoint(insetPoints[j] * profileScale, zPos);
            newPoint[1] += profileYShift;
            newPoint[0] += profileXShift;
            newPoints[j] = newPoint;
            points[profileBegin + outerSize + j] = newPoint;
        }
    }

    
    int adjSize = numProfiles*profileSize;
    vector<char> profileAdjMatrix = partProfile.adjacencyMatrix;


    //Initialises adjacency matrix
    adjMatrix.resize(adjSize*adjSize);
    for(int i = 0; i < adjSize; i++){
        //Fills with zeros initially
        #pragma omp simd
        for(int j = 0; j < adjSize; j++){
            adjMatrix[i*adjSize + j] = 0;
        }
    }


    //Adds profile adjacency matrix to adjacency matrix
    for(int p = 0; p < numProfiles; p++){
        int startI = p*profileSize;

        #pragma omp simd collapse(2)
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
    #pragma omp simd
    for(int i = profileSize; i < profileSize*numProfiles; i++){
        adjMatrix[(i-profileSize)*adjSize + i] = 1;
        adjMatrix[i*adjSize + (i-profileSize)] = 1;
    }

    //Splits rectangles into triangles
    #pragma omp simd collapse(3)
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