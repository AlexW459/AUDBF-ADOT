#include <iostream>
#include "profile.h"


profile::profile(vector<glm::dvec2>& _vertexCoords, double _inset){
    vertexCoords = _vertexCoords;

    int numVerts = vertexCoords.size();


    //Adds inset points to profile
    //Checks for invalid inset
    if (_inset <=0){
        throw runtime_error("Profile inset is less than or equal to zero");
    }
    inset = _inset;

    insetCoords =  generateInset(vertexCoords);


    int totalVerts = numVerts*2;
    //Adds adjacencies to adjacency matrix
    adjacencyMatrix.resize(totalVerts*totalVerts);

    for(int i = 0; i < totalVerts; i++){
        //Fill with zeroes initially
        for(int j = 0; j < totalVerts; j++){
            adjacencyMatrix[i*totalVerts + j] = 0;
        }
    }

    //Adds initial boundary connections to matrix
    for(int i = 0; i < numVerts-1; i++){
        //Outer points
        adjacencyMatrix[(i + 1)*totalVerts + i] = 1;
        adjacencyMatrix[i*totalVerts + (i + 1)] = 1;

        //Inset points
        adjacencyMatrix[(numVerts + i + 1)*totalVerts + (numVerts + i)] = 1;
        adjacencyMatrix[(numVerts + i)*totalVerts + (numVerts + i + 1)] = 1;
    }

    //End to start connection for outer points
    adjacencyMatrix[(numVerts - 1)*totalVerts + 0] = 1;
    adjacencyMatrix[(0*numVerts) + (numVerts - 1)] = 1;
    //Also for inset points
    adjacencyMatrix[(2 * numVerts - 1)*totalVerts + numVerts] = 1;
    adjacencyMatrix[numVerts*totalVerts + (2 * numVerts - 1)] = 1;

    //Connects each inset vertex to its corresponing outer vertex, 
    //as well as the next out vertex
    for(int i = 0; i < numVerts-1; i++){
        adjacencyMatrix[i*totalVerts + (numVerts+i)] = 1;
        adjacencyMatrix[(numVerts+i)*totalVerts + i] = 1;

        adjacencyMatrix[(i+1)*totalVerts + (numVerts+i)] = 1;
        adjacencyMatrix[(numVerts+i)*totalVerts + (i+1)] = 1;
    }

    adjacencyMatrix[(numVerts-1)*totalVerts + (2*numVerts-1)] = 1;
    adjacencyMatrix[(2*numVerts-1)*totalVerts + (numVerts-1)] = 1;

    adjacencyMatrix[0*totalVerts + (2*numVerts-1)] = 1;
    adjacencyMatrix[(2*numVerts-1)*totalVerts + 0] = 1;

    numTriangles = totalVerts;

}

vector<glm::dvec2> profile::generateInset(const vector<glm::dvec2>& outerPoints) const{

    int numVerts = vertexCoords.size();

    //Finds vectors perpendicular to every edge in the polygon, pointing inwards
    vector<glm::dvec2> perpVectors;
    perpVectors.resize(numVerts);
    //Rotates edge 90 degrees
    perpVectors[numVerts-1] = glm::vec2(-(vertexCoords[numVerts-1][1] - vertexCoords[0][1]), 
        vertexCoords[numVerts-1][0] - vertexCoords[0][0]);

    //Normalise vector
    perpVectors[numVerts-1] /= sqrt(dot(perpVectors[numVerts-1], perpVectors[numVerts-1]));
    for(int i = 0; i < numVerts-1; i++){
        perpVectors[i] = glm::dvec2(-(vertexCoords[i][1] - vertexCoords[i+1][1]), 
            vertexCoords[i][0] - vertexCoords[i+1][0]);
        perpVectors[i] /= sqrt(dot(perpVectors[i], perpVectors[i]));
    }
    vector<glm::dvec2> newInsetCoords;
    //Finds vectors from each point to its corresponding inset point
    //by averaging the direction of the perp vectors on either side of the point
    glm::dvec2 insetVector = perpVectors[numVerts-1] + perpVectors[0];
    insetVector *= inset/sqrt(dot(insetVector, insetVector));
    newInsetCoords.resize(numVerts);
    newInsetCoords[0] = vertexCoords[0] + insetVector;

    for(int i = 1; i < numVerts; i++){
        insetVector = perpVectors[i-1] + perpVectors[i];
        insetVector *= inset/sqrt(dot(insetVector, insetVector));
        newInsetCoords[i] = (vertexCoords[i] + insetVector);
    }

    return newInsetCoords;
}

profile::profile(vector<glm::dvec2>& _vertexCoords){
    inset = 0.0f;
    vertexCoords = _vertexCoords;
    int numVerts = vertexCoords.size();

    //Finds triangulation
    adjacencyMatrix.resize(numVerts*numVerts);
    numTriangles = triangulatePolygon(vertexCoords, adjacencyMatrix);

}


profile::profile(){};

profile& profile::operator=(profile& Profile){

    if (this == &Profile)
        return *this;

    vertexCoords = Profile.vertexCoords;
    insetCoords = Profile.insetCoords;
    adjacencyMatrix = Profile.adjacencyMatrix;
    inset = Profile.inset;
    numTriangles = Profile.numTriangles;

    return *this;
}

void profile::plot(int SCREEN_WIDTH, int SCREEN_HEIGHT) const{
    meshWindow window(SCREEN_WIDTH, SCREEN_HEIGHT);

    vector<glm::dvec2> points;
    int numInnerPoints = vertexCoords.size();
    points.resize(numInnerPoints + insetCoords.size());
    //points.insert(points.end(), insetCoords.begin(), insetCoords.end());

    for(int i = 0; i < numInnerPoints; i++){
        points[i] = vertexCoords[i];
    }

    for(int i = 0; i < (int)insetCoords.size(); i++){
        points[i + numInnerPoints] = insetCoords[i];
    }

    window.draw2D(points, adjacencyMatrix);
}

bool profile::pointInTriangle(glm::dvec2 point, glm::dvec2 vert1, glm::dvec2 vert2, glm::dvec2 vert3) const{

    double d1 = (point[0] - vert2[0]) * (vert1[1] - vert2[1]) - (vert1[0] - vert2[0]) * (point[1] - vert2[1]);
    double d2 = (point[0] - vert3[0]) * (vert2[1] - vert3[1]) - (vert2[0] - vert3[0]) * (point[1] - vert3[1]);
    double d3 = (point[0] - vert1[0]) * (vert3[1] - vert1[1]) - (vert3[0] - vert1[0]) * (point[1] - vert1[1]);

    return (d1 <= 0 && d2 <= 0 && d3 <= 0) || (d1 >= 0 && d2 >= 0 && d3 >= 0);
}

//Divides a polygon defined by its vertices (in clockwise order) into triangles, described by an adjacency matrix
int profile::triangulatePolygon(vector<glm::dvec2> vertexCoords, vector<char>& adjacencyMatrix) const{

    int numVerts = vertexCoords.size();
    int adjSize = numVerts;
    //Stores indices of vertices even when some have been removed
    vector<int> vertexIndices;
    vertexIndices.resize(numVerts);


    for(int i = 0; i < adjSize; i++){
        //Fill with zeroes initially
        for(int j = 0; j < adjSize; j++){
            adjacencyMatrix[i*adjSize + j] = 0;
        }
        //Fill vector of indices
        vertexIndices[i] = i;
    }

    

    //Adds initial boundary connections to matrix
    for(int i = 0; i < adjSize-1; i++){
        adjacencyMatrix[(i+1)*adjSize + i] = 1;
        adjacencyMatrix[i*adjSize + (i+1)] = 1;
    }


    adjacencyMatrix[(adjSize-1)*adjSize + 0] = 1;
    adjacencyMatrix[0*adjSize + (adjSize-1)] = 1;


    //numTriangles starts as 1 to account for the final triangle at the end
    //that does not get counted
    int numTriangles = 1;

    //Loop over all vertices and check if any other vertices
    // are inside traignle formed by 3 adjacent vertices
    while (numVerts > 3){

        //Stores ears found in current iteration
        glm::ivec2 earFound(0, 0);

        //Checks if ear has been found
        for(int i = 0; i < numVerts; i++){

            int prevIndex;
            int nextIndex;

            if(i == 0){
                prevIndex = vertexCoords.size()-1;
                nextIndex = i+1;
            }else if (i == (int)vertexCoords.size()-1){
                nextIndex = 0;
                prevIndex = i-1;
            }else{
                prevIndex = i-1;
                nextIndex = i+1;
            }

            //Checks if vertex is reflex by checking if third point lies on the same side of the
            //line segment produced by the first two points as the rest of the polygon
            glm::dvec2 leftDir = glm::dvec2(-(vertexCoords[prevIndex][1] - vertexCoords[i][1]),
                vertexCoords[prevIndex][0]  - vertexCoords[i][0]);
            
            //Is positive if angle is not reflex
            float notReflex = glm::dot(vertexCoords[nextIndex]-vertexCoords[i], leftDir);
            
            if(notReflex > 0){

                //Checks if any of the other points are in the triangle
                int pointCount = 0;
                for(int p = 0; p < numVerts; p++){
                    if(p != i && p != prevIndex && p != nextIndex){
                        //Add 1 if point is found inside triangle
                        pointCount += (int)pointInTriangle(vertexCoords[p], vertexCoords[prevIndex], 
                            vertexCoords[i], vertexCoords[nextIndex]);
         
                    }
                }

                if(pointCount == 0){
                    //Record ear position
                    earFound = glm::ivec2(prevIndex, nextIndex);
                    break;
                }

            }
        }

        //Mark ear
        //Actual indices of ear
        int earIndex1 = vertexIndices[earFound[0]];
        int earIndex2 = vertexIndices[earFound[1]];

        //Add elements to adjacency matrix
        adjacencyMatrix[earIndex1*adjSize + earIndex2] = 1;
        adjacencyMatrix[earIndex2*adjSize + earIndex1] = 1;

        //Record that an ear has been found
        numTriangles++;

        //Remove point from list
        int removePoint;
        if(earFound[1] == 0){
            removePoint = numVerts-1;
        }else{
            removePoint = earFound[1]-1;
        }

        vertexCoords.erase(vertexCoords.begin() + removePoint);
        vertexIndices.erase(vertexIndices.begin() + removePoint);

        numVerts = vertexCoords.size();
    }

    //Adds final adjacencies of remaining points
    int vert1 = vertexIndices[0];
    int vert2 = vertexIndices[1];
    int vert3 = vertexIndices[2];
    adjacencyMatrix[vert1*adjSize + vert2] = 1;
    adjacencyMatrix[vert1*adjSize + vert3] = 1;
    adjacencyMatrix[vert2*adjSize + vert1] = 1;
    adjacencyMatrix[vert2*adjSize + vert3] = 1;
    adjacencyMatrix[vert3*adjSize + vert1] = 1;
    adjacencyMatrix[vert3*adjSize + vert2] = 1;



    //Return triangle count
    return numTriangles;
}

vector<glm::dvec2> generateNACAAirfoil(double maxCamberPercent, double maxCamberPosDecile,
    double maxThicknessPercent, double airfoilChord, double flapRadius, double meshRes){


    //Get actual values from NACA numbers
    double maxCamber = maxCamberPercent/100.0;
    double maxCamberPos = maxCamberPosDecile/10.0;
    double maxThickness = maxThicknessPercent/100.0;
    double curvedLength = (airfoilChord - flapRadius)/airfoilChord;

    //Checks for flat airfoil
    if(maxCamber == 0.0){
        maxCamberPos = 0.5;
    }

    //Gets number of points, accounting for the fact that the airfoil ends in a 
    // single point at both ends
    int numX, numPoints;
    //Increases mesh resolution as airfoils are particularly important
    numX = floor(1.5 * curvedLength * airfoilChord * meshRes) + 1;
    numPoints = 2*numX - 2;


    vector<glm::dvec2> airfoilPoints;
    airfoilPoints.resize(numPoints);

    vector<glm::dvec2> camberPoints;
    camberPoints.resize(numX);
    vector<glm::dvec2> diffXY;
    diffXY.resize(numX); 


    for(int i = 0; i < numX; i++){
        //Gets the x value by finding the current point as a fraction of the total length
        double xVal = pow((double)i/(numX - 1), 1.8);
        //Adjusts for possible reduced length of airfoil
        xVal *= curvedLength;
        camberPoints[i][0] = xVal;

        //Gets thickness of airfoil
        double camberThickness = 5*maxThickness*(0.2969*sqrt(xVal) - 
            0.1260 * xVal - 0.3516*xVal*xVal + 0.2843*xVal*xVal*xVal
            - 0.1036*xVal*xVal*xVal*xVal);


        //Gets y position of centreline
            //Before max camber pos
        if(xVal < maxCamberPos){
            camberPoints[i][1] = maxCamber / (maxCamberPos*maxCamberPos) * 
            (2.0*maxCamberPos*xVal - xVal*xVal) * (xVal < maxCamberPos);
        }else{
            //After max camber pos
            camberPoints[i][1] = maxCamber / ((1.0 - maxCamberPos)*(1.0 - maxCamberPos)) * 
            ((1.0 - 2.0*maxCamberPos) + 2.0*maxCamberPos*xVal - xVal*xVal) * (xVal >= maxCamberPos);
        }

        //Gets gradient perpendicular to airfoil
            //Before max camber pos
        double perpGrad;
        if(maxCamber == 0.0){
            perpGrad = 0.0;

        }else if(xVal < maxCamberPos) {
            perpGrad = 2.0*maxCamber / (maxCamberPos*maxCamberPos) * (maxCamberPos - xVal);
        }else{
            //After max camber pos
            perpGrad = 2.0*maxCamber / ((1.0 - maxCamberPos)*(1.0 - maxCamberPos)) * (maxCamberPos-xVal);
        }

        
        //Gets distance away from camber line in each dimension for upper points
        //(distance for lower points is the negative)
        /*diffXY[i][0] = -camberThickness * (perpGrad / sqrt(1.0 + perpGrad*perpGrad));
        diffXY[i][1] = camberThickness * (1.0 / sqrt(1.0 + perpGrad*perpGrad));*/

        double theta = atan(perpGrad);

        
        diffXY[i][0] =  -camberThickness * sin(theta);
        diffXY[i][1] = camberThickness * cos(theta);

        //cout << "diff: " << diffXY[i][0] << ", " << diffXY[i][1] << endl;

    }


    //Gets airfoil points by adding thickness vectors onto camber line
    for(int i = 0; i < numX - 1; i++){

        //Subtracts half of the airfoil length so that airfoil is centred around the origin
        airfoilPoints[i][0] = camberPoints[i][0] + diffXY[i][0] - curvedLength/2.0;
        airfoilPoints[i][1] = camberPoints[i][1] + diffXY[i][1];

        airfoilPoints[numPoints - numX + 1 + i][0] = camberPoints[numX - i - 1][0] - diffXY[numX - i - 1][0] - curvedLength/2.0;
        airfoilPoints[numPoints - numX + 1 + i][1] = camberPoints[numX - i - 1][1] - diffXY[numX - i - 1][1];

    }

    //Scales all points according to the airfoil chord
    for(int i = 0; i < (int) airfoilPoints.size(); i++){
        airfoilPoints[i] *= airfoilChord;

        //cout << "x: " << airfoilPoints[i][0] << " y: " << airfoilPoints[i][1] << endl;
    }

    return airfoilPoints;
}


vector<glm::dvec2> generateNACAAirfoil(double maxCamberPercent, double maxCamberPosDecile,
    double maxThicknessPercent, double airfoilChord, double meshRes){
        return generateNACAAirfoil(maxCamberPercent, maxCamberPosDecile, maxThicknessPercent, airfoilChord, 0.0, meshRes);
}

