#include "pointPosToIndex.h"

double pointPosToIndex(double minBound, double maxBound, int numIndices, double pointPos){
    double boundSize = maxBound - minBound;

    return (pointPos-minBound)/(boundSize)*((double)numIndices-1.0);
}