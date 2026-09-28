#include "meshIndexTo1DIndex.h"

// SDF is first along x axis, then along y axis, then along z axis
int meshIndexTo1DIndex(int i, int j, int k, int sizeX, int sizeY) {
    return (k * sizeY + j) * sizeX + i;
}
