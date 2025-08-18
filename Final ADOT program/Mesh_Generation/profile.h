#pragma once

#include <vector>
#include <array>
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <stdexcept>

#include "meshWindow.h"

using namespace std;

struct profile{
    profile();

    profile(vector<glm::vec2>& _vertexCoords, double inset);
    profile(vector<glm::vec2>& _vertexCoords);
    //profile(profile& Profile);
    profile& operator=(profile& Profile);


    int triangulatePolygon(vector<glm::vec2> vertexCoords, vector<char>& adjacencyMatrix) const;
    bool pointInTriangle(glm::vec2 point, glm::vec2 vert1, glm::vec2 vert2, glm::vec2 vert3) const;
    vector<glm::vec2> generateInset(const vector<glm::vec2>& outerPoints) const;
    

    void plot(int SCREEN_WIDTH, int SCREEN_HEIGHT);

    //Only contains outer coordinates
    vector<glm::vec2> vertexCoords;
    //Only contains inset coordinates
    vector<glm::vec2> insetCoords;

    vector<char> adjacencyMatrix;

    double inset;

    int numTriangles;
};