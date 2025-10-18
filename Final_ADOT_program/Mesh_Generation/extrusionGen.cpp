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

        //if(extrusion.isControl)  cout << "initial part bounding box: " << boundingBox[0][0] << ", " << boundingBox[0][1] << ", " << boundingBox[0][2] << " - "
        //    << boundingBox[1][0] << ", " << boundingBox[1][1] << ", " << boundingBox[1][2] << endl;

        
    //}
    //else{
    /*if(extrusion.isControl){
        //Start and end points of cylinder encompassing full range of motion of control surface
        glm::dmat2x3 axisEndPoints;

        glm::dvec3 d = extrusion.controlAxis;
        glm::dvec3 x1 = extrusion.pivotPoint;
        //glm::dvec3 x2 = x1 + d;


        double maxDist2 = 0;
        double minT = 10;
        double maxT = -10;

        //Only checks outer points
        for(int i = 0; i < numProfiles; i++){
            for(int j = 0; j < outerSize; j++){
                int index = i*profileSize + j;

                //cout << "point: " << points[index][0] << ", " << points[index][1] << ", " << points[index][2] << endl;
                
                //Gets distance from point to axis of rotation
                //double dist2 = glm::length2(glm::cross(points[index] - x1, points[index] - x2));
                double t = -glm::dot(x1 - points[index], d);

                double dist2 = glm::length2(points[index] - (x1 + d*t));

                //cout << "closest point: " << (x1+d*t)[0] << ", " << (x1+d*t)[1] << ", " << (x1+d*t)[2] << endl;

                //Finds min and max values
                maxDist2 += (dist2 - maxDist2) * (dist2 > maxDist2);
                minT += (t - minT) * (t < minT);
                maxT += (t - maxT) * (t > maxT);

            }
        }

        double maxDist = sqrt(maxDist2);
        axisEndPoints[0] = x1 + minT * d;
        axisEndPoints[1] = x1 + maxT * d;

        //cout << "maxdist: " << maxDist << endl;
        //cout << "minT: " << minT << ", maxT: " << maxT << endl;
        //cout << "axis end points: " << axisEndPoints[0][0] << ", " << axisEndPoints[0][1] << ", " <<
        //axisEndPoints[0][2] << " - " << axisEndPoints[1][0] << ", " << axisEndPoints[1][1] << ", " <<
        //axisEndPoints[1][2] << endl;

        //Gets adjustments that need to be made to each face of the bounding box
        glm::dvec3 boundVec;
        //Relative distance to be moved by each face is the dot product of the 
        //rotation axis and the projection of the rotation axis onto that face
        boundVec[0] = glm::dot(d, glm::dvec3(0, d[1], d[2]));
        boundVec[1] = glm::dot(d, glm::dvec3(d[0], 0, d[2]));
        boundVec[2] = glm::dot(d, glm::dvec3(d[0], d[1], 0));

        //cout << "boundvec: " << boundVec[0] << ", " << boundVec[1] << ", " << boundVec[2] << endl;

        boundingBox[0] = min(axisEndPoints[0], axisEndPoints[1]) - boundVec * maxDist;
        boundingBox[1] = max(axisEndPoints[0], axisEndPoints[1]) + boundVec * maxDist;


        //cout << "initial part bounding box: " << boundingBox[0][0] << ", " << boundingBox[0][1] << ", " << boundingBox[0][2] << " - "
        //    << boundingBox[1][0] << ", " << boundingBox[1][1] << ", " << boundingBox[1][2] << endl;
    }*/


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