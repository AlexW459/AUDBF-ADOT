#include "extrusion.h"

using namespace std;

glm::dmat2x3 extrusion::computeBoundingBox(){
    glm::dmat2x3 boundingBox(INF, INF, INF, -INF, -INF, -INF);

    int numVerts = verts.size();
    for(int v = 0; v < numVerts; v++){
        glm::dvec3 point = verts[v];
        boundingBox[0] = min(boundingBox[0], point);
        boundingBox[1] = max(boundingBox[1], point);
    }

    return boundingBox;
}

glm::dmat2x3 extrusion::computeBoundingBox(glm::dvec3 axis, glm::dvec3 pivotPoint){
    glm::dmat2x3 boundingBox = glm::dmat2x3(INF, INF, INF, -INF, -INF, -INF);



    double minT = INF;
    double maxT = -INF;
    double maxDist = 0.0;

    // Vector perpendicular to the axis of rotation, found by finding perpendicular vector
    // from axis to point
    glm::dvec3 randomPoint = glm::dvec3(10.0, 11.0, 12.0);
    glm::dvec3 perpToAxis = glm::normalize(randomPoint - (pivotPoint + axis*glm::dot(axis, randomPoint - pivotPoint)));

    int numVerts = verts.size();
    for(int v = 0; v < numVerts; v++){
        glm::dvec3 point = verts[v];
        
        // Gets position of closest point on rotation axis
        double t = glm::dot(axis, point - pivotPoint);
        glm::dvec3 pointOnAxis = pivotPoint + axis*t;
        glm::dvec3 segToPoint = point-pointOnAxis;
        double dist = glm::length(segToPoint);
        //cout << "Point: " << point[0] << ", " << point[1] << ", " << point[2] << endl;
        
        //cout << "segToPoint: " << segToPoint[0] << ", " << segToPoint[1] << ", " << segToPoint[2] << endl;

        maxDist = max(dist, maxDist);
        minT = min(t, minT);
        maxT = max(t, maxT);
    }

    // Gets second perpendicular vector
    glm::dvec3 perpToAxis2 = glm::cross(axis, perpToAxis);

    // Gets maximum and minimum points in each cartesian direction from each end point
    glm::dvec3 p0 = pivotPoint + axis*minT;
    glm::dvec3 p1 = pivotPoint + axis*maxT;

    // Gets values of theta that maximise/minimise each cartesian coordinate
    glm::dvec3 thetaVals(0.0);
    for(int i = 0; i < 3; i++){
        if(fabs(perpToAxis[i]) < 1e-12 && fabs(perpToAxis2[i]) < 1e-12) thetaVals[i] = 0.0;
        else if(fabs(perpToAxis[i]) < 1e-12) thetaVals[i] = M_PI/2.0;
        else if(fabs(perpToAxis2[i]) < 1e-12) thetaVals[i] = 0.0;
        else thetaVals[i] = atan(perpToAxis2[i]/perpToAxis[i]);
    }

    // Gets bounding box
    glm::dvec3 v1 = maxDist*perpToAxis*cos(thetaVals);
    glm::dvec3 v2 = maxDist*perpToAxis2*sin(thetaVals);
    boundingBox[0] = min(p0 + v1 + v2, boundingBox[0]);
    boundingBox[0] = min(p0 - v1 - v2, boundingBox[0]);
    boundingBox[0] = min(p1 + v1 + v2, boundingBox[0]);
    boundingBox[0] = min(p1 - v1 - v2, boundingBox[0]);
    boundingBox[1] = max(p0 + v1 + v2, boundingBox[1]);
    boundingBox[1] = max(p0 - v1 - v2, boundingBox[1]);
    boundingBox[1] = max(p1 + v1 + v2, boundingBox[1]);
    boundingBox[1] = max(p1 - v1 - v2, boundingBox[1]);

    /*cout << "part bounding box: " << boundingBox[0][0] << ", " << boundingBox[0][1] << ", "
        << boundingBox[0][2] << ", " << boundingBox[1][0] << ", " << boundingBox[1][1] << ", "
        << boundingBox[1][2] << endl;*/

    return boundingBox;
}