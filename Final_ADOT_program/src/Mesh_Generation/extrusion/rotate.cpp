#include "extrusion.h"

void extrusion::rotate(glm::dquat q){
    int numVerts = verts.size();
    for(int v = 0; v < numVerts; v++){
        verts[v] = q * verts[v];
    }
}