#include "MULEplane.h"

void calcDerivedParams(vector<string>&, vector<double>&){
    return;
}

profile fuselageProfile(vector<string> paramNames, vector<double> paramVals, double meshRes){

    double fuselageHeight = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "fuselageHeight") - paramNames.begin())];
    double fuselageParam1 = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "fuselageParam1") - paramNames.begin())];
    double fuselageParam2 = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "fuselageParam2") - paramNames.begin())];
    //double plasticThickness = 0.05;

    //Defines points that set bezier curve
    glm::vec2 p1(0, fuselageHeight*0.5);
    glm::vec2 p2(fuselageParam1, fuselageHeight*0.5);
    glm::vec2 p3(fuselageParam2, -fuselageHeight*0.5);
    glm::vec2 p4(0, -fuselageHeight*0.5);

    int numPoints = ceil((fuselageHeight + fuselageParam1 + fuselageParam2) / 3 * M_PI * meshRes);

    vector<glm::vec2> bezPoints;
    bezPoints.resize(numPoints*2);

    glm::vec2 p2f(p2[0], p2[1]);
    glm::vec2 p3f(p3[0], p3[1]);

    //Finds points on bezier curve
    for(int i = 0; i < numPoints; i++){
        float tVal = (float)i/numPoints;
        bezPoints[i] = (1 - tVal)*(1 - tVal)*(1 - tVal) * p1 + tVal * (3 * (1-tVal)*(1-tVal)) * p2
                     + (3 * (1-tVal) * tVal*tVal) * p3 + tVal*tVal*tVal * p4;
        bezPoints[i+numPoints] = -1.0f*((1 - tVal)*(1 - tVal)*(1 - tVal) * p1 + tVal * (3 * (1-tVal)*(1-tVal)) * p2
                     + (3 * (1-tVal) * tVal*tVal) * p3 + tVal*tVal*tVal * p4);
    }

    bezPoints = {glm::vec2(0, 0), glm::vec2(0, 1), glm::vec2(0.5, 1.5), glm::vec2(1, 1), glm::vec2(1,0)};

    profile outputProfile(bezPoints);

    return outputProfile;
}


extrusionData extrudeFuselage(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double fuselageLength = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "fuselageLength") - paramNames.begin())];

    extrusionData extrusion;

    extrusion.scaleVals = {1.0f, 1.0f};
    extrusion.yPosVals = {0.0f, 0.0f};
    extrusion.zSampleVals = {0.0f, (float)fuselageLength};
    extrusion.zSampleVals = {0.0f, 1.0f};
    extrusion.sweep = 0.0f;
    extrusion.translation = glm::vec3(0.0f, 0.0f, 0.0f);

    extrusion.rotation = glm::quat(glm::vec3(0.5f*M_PI, 0.0f, -0.5f*M_PI));
    //extrusion.rotation = glm::quat(glm::vec3(0.0f, 0.0f, 0.0f));
    

    return extrusion;
}