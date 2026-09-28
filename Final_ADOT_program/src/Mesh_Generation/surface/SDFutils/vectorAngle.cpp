#include "vectorAngle.h"

double vectorAngle(glm::dvec3 a, glm::dvec3 b){
    double cosAngle = glm::dot(glm::normalize(a), glm::normalize(b));

    if(cosAngle < -1.0) cosAngle = -1.0;
    else if (cosAngle > 1.0) cosAngle = 1.0;
    return acosf64(cosAngle);
}