#pragma once

#include <vector>
#include <string>
#include <glm/glm.hpp>


#include "../../Mesh_Generation/extrusion/extrusion.h"



// Finds the variables associated with the volumetric mesh, including the bounding box of each part
void getVolVals(const extrusion& partExtrusion, std::vector<glm::dvec3> partPointMassLocations, 
    std::vector<double> pointMasses, double density, double& mass, glm::dvec3& COM, 
    glm::dmat3& MOI);
        