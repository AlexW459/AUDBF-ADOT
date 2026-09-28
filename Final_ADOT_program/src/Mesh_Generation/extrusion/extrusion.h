#pragma once

#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <vector>
#include <glm/glm.hpp>
#include <algorithm>
#include <limits>
#include <iostream>
#include "../profile/profile.h"
#include "extrusionData.h"
#include "../../../compile_config.h"

#define INF 20.0//numeric_limits<double>::infinity()

struct extrusion{
    // Details of mesh
    std::vector<glm::dvec3> verts;
    // Edges list does not include internal edges of end profiles as they do not need to be
    // tested during SDF generation
    std::vector<glm::ivec2> edges;
    std::vector<glm::ivec3> faces;
    std::vector<glm::dvec3> faceNormals;
    std::vector<glm::ivec4> tets;

    // Additional details of mesh required for SDF gen
    // Faces adjacent to each vertex
    std::vector<std::vector<int>> vertAdjFaces;
    // Faces adjacent to each edge
    std::vector<glm::ivec2> edgeAdjFaces;

    // Adjacency matrix of vertices
    std::vector<char> adjMatrix;

    // Gets extrusion from data
    extrusion(const profile& partProfile, const extrusionData& extrusioInfon);
    extrusion();

    inline extrusion& operator=(extrusion Extrusion){
        if (this == &Extrusion)return *this;

        verts = Extrusion.verts;
        edges = Extrusion.edges;
        faces = Extrusion.faces;
        tets = Extrusion.tets;
        faceNormals = Extrusion.faceNormals;
        vertAdjFaces = Extrusion.vertAdjFaces;
        edgeAdjFaces = Extrusion.edgeAdjFaces;
        adjMatrix = Extrusion.adjMatrix;
        
        return *this;
    }

    void translate(glm::dvec3 p);
    void rotate(glm::dquat q);

    // Finds normals of faces
    void computeNormals();
    // Finds bounding box in cartesian coordinates
    glm::dmat2x3 computeBoundingBox();
    // Bounding box of control part
    glm::dmat2x3 computeBoundingBox(glm::dvec3 axis, glm::dvec3 pivotPoint);

    // Plots to the screen
    #ifdef USE_SDL
        void plot(int WINDOW_WIDTH, int WINDOW_HEIGHT, profile partProfile) const;
    #endif
};