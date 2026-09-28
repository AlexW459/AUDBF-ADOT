#pragma once

#include <functional>
#include <vector>
#include <string>

#include "../../Mesh_Generation/profile/profile.h"

typedef std::function<profile(std::vector<std::string>, 
        std::vector<double>, double)> profileFuncType;