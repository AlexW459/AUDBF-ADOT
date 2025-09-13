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


    float extrusionBegin = *min_element(extrusion.zSampleVals.begin(), extrusion.zSampleVals.end());

    float extrusionEnd = *max_element(extrusion.zSampleVals.begin(), extrusion.zSampleVals.end());

    //Finds positions of profiles
    for(int i = 0; i < numProfiles; i++){
        float zPos = extrusion.zSampleVals[i];
        int profileBegin = i*profileSize;

        glm::dvec3 profileShift = glm::dvec3(extrusion.posVals[i], 0);
        glm::dvec2 profileScale = extrusion.scaleVals[i];
        #pragma omp simd
        for(int j = 0; j < outerSize; j++){
            //Shifts and scales points
            glm::dvec3 newPoint = glm::dvec3(partProfile.vertexCoords[j] * profileScale, zPos) + profileShift;
            newPoints[j] = newPoint;
            points[profileBegin + j] = newPoint;

        }

        //Adds inset points
        vector<glm::dvec2> insetPoints = partProfile.generateInset(newPoints);
        //Loop does not run if there is no inset
        #pragma omp simd
        for(int j = 0; j < profileSize-outerSize; j++){
            glm::dvec3 newPoint = glm::dvec3(insetPoints[j] * profileScale, zPos) + profileShift;
            points[profileBegin + outerSize + j] = newPoint;
        }

    }

    //Gets maximum distance from points to axis of rotation and the endpoints of the smallest cylinder
    //that encloses all of the points

    if(!extrusion.isControl){

        //Initialises bounding box
        boundingBox[0][2] = extrusionBegin;
        boundingBox[1][2] = extrusionEnd;

        boundingBox[0][0] = 50;
        boundingBox[0][1] = 50;
        boundingBox[1][0] = -50;
        boundingBox[1][1] = -50;


        //Only checks outer points
        #pragma omp simd collapse(2)
        for(int i = 0; i < numProfiles; i++){
            for(int j = 0; j < outerSize; j++){

                //Updates bounding box with new minimums and maximums
                boundingBox[0][0] += (points[i][0] - boundingBox[0][0]) *  (points[i][0] < boundingBox[0][0]);
                boundingBox[0][1] += (points[i][1] - boundingBox[0][1]) *  (points[i][1] < boundingBox[0][1]);
                boundingBox[1][0] += (points[i][0] - boundingBox[1][0]) *  (points[i][0] > boundingBox[1][0]);
                boundingBox[1][1] += (points[i][1] - boundingBox[1][1]) *  (points[i][1] > boundingBox[1][1]);

            }
        }
    }
    else{
        double minDist;
        glm::dmat2x3 axisEndPoints;

        glm::dvec3 d = extrusion.controlAxis;
        glm::dvec3 b = extrusion.pivotPoint;
        //Start and end points of cylinder encompassing full range of motion of control surface
        glm::dvec3 beginPoint;
        glm::dvec3 endPoint;

        double minDist2 = 10;
        double minT = 10;
        double maxT = -10;

        //Only checks outer points
        #pragma omp simd collapse(2)
        for(int i = 0; i < numProfiles; i++){
            for(int j = 0; j < outerSize; j++){
                int index = i*profileSize + j;
                
                //Gets closest point on axis of rotation
                glm::dvec3 v = points[index] - b;
                double t = glm::dot(d, v);
                glm::dvec3 axisPoint = b + t * d;

                //Gets distance from point to closest point on axis
                double dist2 = glm::dot(points[index] - axisPoint, points[index] - axisPoint);

                //Finds min and max values
                minDist2 += (dist2 - minDist2) * (dist2 < minDist2);
                minT += (t - minT) * (t < minT);
                maxT += (t - maxT) * (t > maxT);
            }
        }

        minDist = sqrt(minDist2);
        axisEndPoints[0] = b + minT * d;
        axisEndPoints[1] = b + maxT * d;

        //Gets adjustments that need to be made to each face of the bounding box
        glm::dvec3 boundVec;
        //Relative istance to be moved by each face the dot product of the 
        //rotation axis and the rotation axis and its projection onto that face
        boundVec[0] = glm::dot(d, glm::dvec3(0, d[1], d[2]) / sqrt(d[1] * d[1] + d[2] * d[2]));
        boundVec[1] = glm::dot(d, glm::dvec3(d[0], 0, d[2]) / sqrt(d[0] * d[0] + d[2] * d[2]));
        boundVec[2] = glm::dot(d, glm::dvec3(d[0], d[1], 0) / sqrt(d[0] * d[0] + d[1] * d[1]));

        boundingBox[0] = min(axisEndPoints[0], axisEndPoints[1]) - boundVec * minDist;
        boundingBox[1] = max(axisEndPoints[0], axisEndPoints[1]) + boundVec * boundVec;

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