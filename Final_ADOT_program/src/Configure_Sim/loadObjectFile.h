#pragma once

#include <string>
#include <dlfcn.h>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <memory>

#include "../Score_Evaluation/testModel/testModel.h"

void* loadObjectFile(std::string fileName, void*& handle);