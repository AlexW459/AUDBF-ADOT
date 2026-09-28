#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <iostream>


inline bool pointInTriangle(glm::dvec2 point, glm::dvec2 vert1, glm::dvec2 vert2, glm::dvec2 vert3){

    double d1 = (point[0] - vert2[0]) * (vert1[1] - vert2[1]) - (vert1[0] - vert2[0]) * (point[1] - vert2[1]);
    double d2 = (point[0] - vert3[0]) * (vert2[1] - vert3[1]) - (vert2[0] - vert3[0]) * (point[1] - vert3[1]);
    double d3 = (point[0] - vert1[0]) * (vert3[1] - vert1[1]) - (vert3[0] - vert1[0]) * (point[1] - vert1[1]);

    return (d1 <= 0.0 && d2 <= 0.0 && d3 <= 0.0) || (d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0);
}

/* Divides a polygon defined by its vertices into triangles, described by a list of vertex 
indices and an adjacency matrix. Requires points are clockwise*/
inline int triangulatePolygon(std::vector<glm::dvec2> vertexCoords, 
    std::vector<char>& adjacencyMatrix, std::vector<glm::ivec3>& trianglesFound){

    int numVerts = vertexCoords.size();
    int adjSize = numVerts;
    //Stores indices of vertices even when some have been removed
    std::vector<int> vertexIndices(numVerts);

    // Triangulation is written assuming points are clockwise in the x-y plane,
    // And so points are flipped if this is not the case
    adjacencyMatrix.resize(adjSize*adjSize, 0);
    //std::cout << "points" << std::endl;

    //std::vector<glm::dvec2> verts = vertexCoords;

    for(int i = 0; i < adjSize; i++){
        //std::cout << i << ": " << vertexCoords[i][0] << ", " << vertexCoords[i][1] << std::endl;

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


    //triangleNum starts as 1 to account for the final triangle at the end
    //that does not get counted
    int triangleNum = 1;
    // Gets number of triangles using rule
    int numTriangles = numVerts-2;
    trianglesFound.resize(numTriangles);


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
                vertexCoords[prevIndex][0] - vertexCoords[i][0]);
            
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
                    //cout << "ear found: " << prevIndex << ", " << nextIndex << endl;
                    break;
                }

            }
        }


        //Mark ear
        //Actual indices of ear
        int earIndex1 = vertexIndices[earFound[0]];
        int earIndex2 = vertexIndices[earFound[1]];

        //int correctedIndex1 = isClockwise ? earIndex1 : adjSize - 1 - earIndex1;
        //int correctedIndex2 = isClockwise ? earIndex2 : adjSize - 1 - earIndex2;


        //Add elements to adjacency matrix
        //adjacencyMatrix[correctedIndex1*adjSize + correctedIndex2] = 1;
        //adjacencyMatrix[correctedIndex2*adjSize + correctedIndex1] = 1;

        adjacencyMatrix[earIndex1*adjSize + earIndex2] = 1;
        adjacencyMatrix[earIndex2*adjSize + earIndex1] = 1;

        //Get index of point to remove
        int removePoint;
        if(earFound[1] == 0){
            removePoint = numVerts-1;
        }else{
            removePoint = earFound[1]-1;
        }
        int removeIndex = vertexIndices[removePoint];
        //int correctedRemovePoint = isClockwise ? vertexIndices[removePoint] : 
        //    adjSize - 1 - vertexIndices[removePoint];


        //cout << "adding triangle " << triangleNum << endl;


        trianglesFound[triangleNum-1] = glm::ivec3(earIndex1, 
            removeIndex, earIndex2);
        
        /*std::cout << "triangle: " << verts[trianglesFound[triangleNum-1][0]][0] << ", " <<
            verts[trianglesFound[triangleNum-1][0]][1] << "     ";
        std::cout << verts[trianglesFound[triangleNum-1][1]][0] << ", " << 
            verts[trianglesFound[triangleNum-1][1]][1] << "     ";
        std::cout << verts[trianglesFound[triangleNum-1][2]][0] << ", " << 
            verts[trianglesFound[triangleNum-1][2]][1] << std::endl;*/

        //std::cout << "triangle: " << trianglesFound[triangleNum-1][0] << ", " <<
        //    trianglesFound[triangleNum-1][1] << ", " << trianglesFound[triangleNum-1][2] << std::endl;

        vertexCoords.erase(vertexCoords.begin() + removePoint);
        vertexIndices.erase(vertexIndices.begin() + removePoint);

        numVerts = vertexCoords.size();

        //Record that an ear has been found
        triangleNum++;
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

    //cout << "adding last triangle" << endl;

    // Add final triangle
    trianglesFound[triangleNum-1] = glm::ivec3(vert1, vert2, vert3);

    //cout << "triangle: " << trianglesFound[triangleNum-1][0] << " " << trianglesFound[triangleNum-1][1] << " " << trianglesFound[triangleNum-1][2] << endl;

    //Return triangle count
    return numTriangles;
}