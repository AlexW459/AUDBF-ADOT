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

    profile(vector<glm::dvec2>& _vertexCoords, double inset);
    profile(vector<glm::dvec2>& _vertexCoords);
    //profile(profile& Profile);
    profile& operator=(profile& Profile);


    int triangulatePolygon(vector<glm::dvec2> vertexCoords, vector<char>& adjacencyMatrix) const;
    bool pointInTriangle(glm::dvec2 point, glm::dvec2 vert1, glm::dvec2 vert2, glm::dvec2 vert3) const;
    vector<glm::dvec2> generateInset(const vector<glm::dvec2>& outerPoints) const;
    

    void plot(int WINDOW_WIDTH, int WINDOW_HEIGHT) const;

    //Only contains outer coordinates
    vector<glm::dvec2> vertexCoords;
    //Only contains inset coordinates
    vector<glm::dvec2> insetCoords;

    vector<char> adjacencyMatrix;

    double inset;

    int numTriangles;
};

vector<glm::dvec2> generateNACAAirfoil(double maxCamberPercent, double maxCamberPosDecile,
    double maxThicknessPercent, double airfoilChord, double flapRadius, double meshRes);

vector<glm::dvec2> generateNACAAirfoil(double maxCamberPercent, double maxCamberPosDecile,
    double maxThicknessPercent, double airfoilChord, double meshRes);
