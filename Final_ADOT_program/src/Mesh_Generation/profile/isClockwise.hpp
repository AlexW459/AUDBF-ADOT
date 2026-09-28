#pragma once

#include <vector>
#include <glm/glm.hpp>


inline double isClockwise(std::vector<glm::dvec2> vertexCoords){

    int numVerts = vertexCoords.size();
    double sumSoFar = 0;
    for(int i = 0; i < numVerts; i++){
        glm::dvec2 point1 = vertexCoords[i];
        glm::dvec2 point2 = i == numVerts-1 ? vertexCoords[0] : vertexCoords[i+1];

        double xDiff = point2[0] - point1[0];
        double ySum = point1[1] + point2[1];

        sumSoFar += xDiff * ySum;
    }

    return sumSoFar;
}