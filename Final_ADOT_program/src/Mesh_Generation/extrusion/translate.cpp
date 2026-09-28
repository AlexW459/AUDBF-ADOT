#include "extrusion.h"

void extrusion::translate(glm::dvec3 p){
    int numVerts = verts.size();
    for(int v = 0; v < numVerts; v++){
        verts[v] += p;
    }
}