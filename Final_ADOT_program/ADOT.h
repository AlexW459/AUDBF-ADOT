#pragma once

#include "src/User_utils/generateNACAairfoil.hpp"
#include "src/User_utils/getParam.hpp"
#include "src/User_utils/readCSV.hpp"
#include "src/Mesh_Generation/extrusion/extrusionData.h"
#include "src/Mesh_Generation/profile/profile.h"
#include "src/Mesh_Generation/marchingCubes/marchingCubes.hpp"
#include "src/Score_Evaluation/testModel/testModel.h"


extern "C" testModel constructModel();