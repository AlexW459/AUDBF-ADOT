#pragma once

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <mpi.h>
#include <random>

#ifndef INCLUDE_MODEL
// Include header file of model here
#include "MULEplaneModel/MULEplane.h"
#define INCLUDE_MODEL
#endif


#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "compile_config.h"
#ifdef USE_SDL
#include "SDL2/SDL.h"
#endif


#include "aircraft.h"
#include "readCSV.h"
#include "Mesh_Generation/meshWindow.h"
#include "parseParameters.h"