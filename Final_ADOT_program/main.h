#pragma once

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <mpi.h>
#include <random>


#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "constants.h"
#ifdef USE_SDL
#include "SDL2/SDL.h"
#endif

#include "readCSV.h"
#include "aircraft.h"
//#include "Mesh_Generation/meshWindow.h"
#include "MULEplaneModel/MULEplane.h"

