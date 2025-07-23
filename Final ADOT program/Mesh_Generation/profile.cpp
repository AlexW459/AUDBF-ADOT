#include <iostream>
#include "profile.h"


profile::profile(vector<glm::vec2>& _vertexCoords, double _inset){
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

vector<glm::vec2> profile::generateInset(const vector<glm::vec2>& outerPoints) const{

    int numVerts = vertexCoords.size();

    //Finds vectors perpendicular to every edge in the polygon, pointing inwards
    glm::vec2* perpVectors = new glm::vec2[numVerts];
    //Rotates edge 90 degrees
    perpVectors[numVerts-1] = glm::vec2(-(vertexCoords[numVerts-1][1] - vertexCoords[0][1]), 
        vertexCoords[numVerts-1][0] - vertexCoords[0][0]);


    //Normalise vector
    perpVectors[numVerts-1] /= sqrt(dot(perpVectors[numVerts-1], perpVectors[numVerts-1]));
    #pragma omp simd
    for(int i = 0; i < numVerts-1; i++){
        
        perpVectors[i] = glm::vec2(-(vertexCoords[i][1] - vertexCoords[i+1][1]), 
            vertexCoords[i][0] - vertexCoords[i+1][0]);
        perpVectors[i] /= sqrt(dot(perpVectors[i], perpVectors[i]));
    }
    vector<glm::vec2> newInsetCoords;
    //Finds vectors from each point to its corresponding inset point
    //by averaging the direction of the perp vectors on either side of the point
    glm::vec2 insetVector = perpVectors[numVerts-1] + perpVectors[0];
    insetVector *= inset/sqrt(dot(insetVector, insetVector));
    newInsetCoords.resize(numVerts);
    newInsetCoords[0] = vertexCoords[0] + insetVector;

    #pragma omp simd
    for(int i = 1; i < numVerts; i++){
        insetVector = perpVectors[i-1] + perpVectors[i];
        insetVector *= inset/sqrt(dot(insetVector, insetVector));
        newInsetCoords[i] = (vertexCoords[i] + insetVector);
    }

    delete[] perpVectors;
    return newInsetCoords;
}

profile::profile(vector<glm::vec2>& _vertexCoords){
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

void profile::plot(int SCREEN_WIDTH, int SCREEN_HEIGHT){
    meshWindow window(SCREEN_WIDTH, SCREEN_HEIGHT);

    vector<glm::vec2> points = vertexCoords;
    points.insert(points.end(), insetCoords.begin(), insetCoords.end());

    window.draw2D(points, adjacencyMatrix);
}

bool profile::pointInTriangle(glm::vec2 point, glm::vec2 vert1, glm::vec2 vert2, glm::vec2 vert3) const{

    float d1 = (point[0] - vert2[0]) * (vert1[1] - vert2[1]) - (vert1[0] - vert2[0]) * (point[1] - vert2[1]);
    float d2 = (point[0] - vert3[0]) * (vert2[1] - vert3[1]) - (vert2[0] - vert3[0]) * (point[1] - vert3[1]);
    float d3 = (point[0] - vert1[0]) * (vert3[1] - vert1[1]) - (vert3[0] - vert1[0]) * (point[1] - vert1[1]);

    return (d1 <= 0 && d2 <= 0 && d3 <= 0) || (d1 >= 0 && d2 >= 0 && d3 >= 0);
}

//Divides a polygon defined by its vertices (in clockwise order) into triangles, described by an adjacency matrix
int profile::triangulatePolygon(vector<glm::vec2> vertexCoords, vector<char>& adjacencyMatrix) const{

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
        glm::vec2 earFound;

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
            glm::vec2 leftDir = glm::vec2(-(vertexCoords[prevIndex][1] - vertexCoords[i][1]),
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
                    earFound = glm::vec2(prevIndex, nextIndex);
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