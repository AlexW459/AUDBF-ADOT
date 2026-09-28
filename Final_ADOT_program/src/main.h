#pragma once

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <mpi.h>
#include <random>


#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#ifdef USE_SDL
#include <SDL2/SDL.h>
#endif

#include "../compile_config.h"
#include "Score_Evaluation/testModel/testModel.h"
#include "Score_Evaluation/calculateScore.h"

#include "Configure_Sim/parseParameters.h"


