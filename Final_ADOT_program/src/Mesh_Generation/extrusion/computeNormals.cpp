#include "extrusion.h"

void extrusion::computeNormals(){
    int numFaces = faces.size();
    for(int t = 0; t < numFaces; t++){
        // Gets normal. points of face are counter-clockwise facing into the extrusion so 
        // so go clockwise
        glm::dvec3 v1 = verts[faces[t][0]];
        glm::dvec3 v2 = verts[faces[t][1]];
        glm::dvec3 v3 = verts[faces[t][2]];
        glm::dvec3 normal = glm::cross(v2-v1, v3-v1);
        faceNormals[t] = glm::normalize(normal);
    }
}