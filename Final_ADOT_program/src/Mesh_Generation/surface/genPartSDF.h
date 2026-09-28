#pragma once

#include <vector>

#include "../extrusion/extrusion.h"

#include "faceSDF.h"
#include "edgeSDF.h"
#include "vertexSDF.h"

//Gets SDF of a single part with a specified resolution. Returns size of SDF
void genPartSDF(extrusion partExtrusion, const std::vector<double>& xVals, 
    const std::vector<double>& yVals, const std::vector<double>& zVals, double bandSize,
    std::vector<double>& SDF);
