#pragma once

#include <iostream>
#include <fstream>

#include "MarchingCubeCpp/MC.h"

using namespace std;

void writeMeshToObj(string fileName, MC::mcMesh mesh);