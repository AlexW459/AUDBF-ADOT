#pragma once

#include <vector>
#include <array>
#include <algorithm>
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <stdexcept>

#include "../../../compile_config.h"
#include "isClockwise.hpp"
#include "triangulatePolygon.hpp"

#ifdef USE_SDL
    #include "../meshWindow/meshWindow.hpp"
#endif


struct profile{

    profile(){};

    inline profile(std::vector<glm::dvec2>& _vertexCoords){
        vertexCoords = _vertexCoords;

        //Finds triangulation

        //Gets winding order
        double order = isClockwise(vertexCoords);
        if(order == 0.0){
            throw std::runtime_error("Invalid point set in profile");
        }else if (order < 0.0){
            // Counter-clockwise

            // Reverse order of points
            std::reverse(vertexCoords.begin(), vertexCoords.end());
        }

        //cout << "triangulating" << endl;
        triangulatePolygon(vertexCoords, adjacencyMatrix, triangles);
        //cout << "finished" << endl;
    }

    /* Generates profile from points. Points must be in counter-clockwise direction 
    when facing into the extrusion (so must be clockwise in x-y plane if extruding 
    in z direction) */
    inline profile& operator=(profile &Profile){
        if (this == &Profile)return *this;

        vertexCoords = Profile.vertexCoords;
        adjacencyMatrix = Profile.adjacencyMatrix;
        triangles = Profile.triangles;
        
        return *this;
    }

    /*inline profile& operator=(profile Profile){
        if (this == &Profile)return *this;

        vertexCoords = Profile.vertexCoords;
        adjacencyMatrix = Profile.adjacencyMatrix;
        triangles = Profile.triangles;
        
        return *this;
    }*/

    
    #ifdef USE_SDL
    inline void plot(int WINDOW_WIDTH, int WINDOW_HEIGHT) const{
        meshWindow window(WINDOW_WIDTH, WINDOW_HEIGHT);

        window.draw2D(vertexCoords, adjacencyMatrix);
    }
    #endif

    std::vector<glm::dvec2> vertexCoords;

    std::vector<char> adjacencyMatrix;
    std::vector<glm::ivec3> triangles;
};

