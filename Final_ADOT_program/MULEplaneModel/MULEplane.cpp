#include "MULEplane.h"

void calcDerivedParams(vector<string>& paramNames, vector<double>& paramVals){
    double fuselageLength = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "fuselageLength") - paramNames.begin())];
    double wingRootChord = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingRootChord") - paramNames.begin())];
    double wingHorizontalPosition = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingHorizontalPosition") - paramNames.begin())];


    double wingXPos = (fuselageLength-wingRootChord)*wingHorizontalPosition+wingRootChord*0.5;
    paramNames.push_back("wingXPos");
    paramVals.push_back(wingXPos);

    return;
}

profile fuselageProfile(vector<string> paramNames, vector<double> paramVals, double meshRes){

    double fuselageHeight = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "fuselageHeight") - paramNames.begin())];

    double fuselageParam1 = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "fuselageParam1") - paramNames.begin())];
    double fuselageParam2 = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "fuselageParam2") - paramNames.begin())];
    double plasticThickness = 0.05;

    //Defines points that set bezier curve
    glm::dvec2 p1(0, fuselageHeight*0.5);
    glm::dvec2 p2(fuselageParam1, fuselageHeight*0.5);
    glm::dvec2 p3(fuselageParam2, -fuselageHeight*0.5);
    glm::dvec2 p4(0, -fuselageHeight*0.5);

    int numPoints = ceil((fuselageHeight + fuselageParam1 + fuselageParam2) / 3 * M_PI * meshRes);


    vector<glm::dvec2> bezPoints;
    bezPoints.resize(numPoints*2 - 2);

    //Finds points on bezier curve
    for(int i = 0; i < numPoints; i++){
        double tVal = (double)i/numPoints;
        bezPoints[i] = (1 - tVal)*(1 - tVal)*(1 - tVal) * p1 + tVal * 3 * (1-tVal)*(1-tVal) * p2
                     + (3 * (1-tVal) * tVal*tVal) * p3 + tVal*tVal*tVal * p4;
    }

    //Copies points to opposite side of profile
    for(int i = 1; i < numPoints - 1; i++){
        bezPoints[2*numPoints - 2 - i] = glm::dvec2(-1.0*bezPoints[i][0], bezPoints[i][1]);
    }


    profile outputProfile(bezPoints, plasticThickness);

    return outputProfile;
}


extrusionData extrudeFuselage(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double fuselageLength = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "fuselageLength") - paramNames.begin())];
    double fuselageHeight = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "fuselageHeight") - paramNames.begin())];
    double noseconeLength = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "noseconeLength") - paramNames.begin())];
    double noseconeTipScale = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "noseconeTipScale") - paramNames.begin())];
    double noseconeOffset = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "noseconeOffset") - paramNames.begin())];
    double tailconeLength = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "tailconeLength") - paramNames.begin())];
    double tailconeTipScale = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "tailconeTipScale") - paramNames.begin())];

    extrusionData extrusion;

    double tailconeTipHeight = tailconeTipScale*fuselageHeight;
    double noseconeTipHeight = noseconeTipScale*fuselageHeight;
    //Finds the length of the uncurved section of the nosecone
    double noseQ2zPos = (noseconeTipHeight*noseconeLength)/(fuselageHeight-0.5*noseconeTipHeight);
    double tailQ2zPos = (tailconeTipHeight*tailconeLength)/(fuselageHeight-0.5*tailconeTipHeight);

    int tailconeNumT = ceil(0.5*meshRes*tailQ2zPos);
    int noseconeNumT = ceil(0.5*meshRes*noseQ2zPos);


    extrusion.zSampleVals.resize(noseconeNumT + 2 + tailconeNumT);
    extrusion.posVals.resize(noseconeNumT + 2 + tailconeNumT);
    extrusion.scaleVals.resize(noseconeNumT + 2 + tailconeNumT);


    glm::dvec2 Q2 = glm::dvec2(-tailconeLength-0.5*tailQ2zPos, fuselageHeight*0.5);
    double tailStraightLength = tailconeLength - 0.5*tailQ2zPos;
    glm::dvec2 Q1 = glm::dvec2(-tailStraightLength, fuselageHeight*0.5-tailconeTipHeight);
    glm::dvec2 Q3 = glm::dvec2(-tailStraightLength, fuselageHeight*0.5);

    for(int i = 0; i < tailconeNumT; i++){
        //Multiply by 0.5 to get tValue as loop simultaneously goes from 0 to 0.5 and 1 to 0.5
        double tVal = (double)i/tailconeNumT*0.5;

        glm::dvec2 lowerPoint = (1-tVal)*(1-tVal)*Q1+2.0*(1-tVal)*tVal*Q2+tVal*tVal*Q3;
        glm::dvec2 upperPoint = tVal*tVal*Q1+2.0*(1-tVal)*tVal*Q2+(1-tVal)*(1-tVal)*Q3;

        //Flips to account for reverse direction
        extrusion.zSampleVals[tailconeNumT - i - 1] = upperPoint[0];
        extrusion.posVals[tailconeNumT - i - 1] = glm::dvec2(0, (upperPoint[1]+lowerPoint[1])/2);
        extrusion.scaleVals[tailconeNumT - i - 1] = glm::dvec2((upperPoint[1]-lowerPoint[1])/fuselageHeight, (upperPoint[1]-lowerPoint[1])/fuselageHeight);

    }


    //Straight section of fuselage
    extrusion.zSampleVals[tailconeNumT] = 0;
    extrusion.zSampleVals[tailconeNumT + 1] = fuselageLength;
    extrusion.posVals[tailconeNumT] = glm::dvec2(0, 0);
    extrusion.posVals[tailconeNumT + 1] = glm::dvec2(0, 0);
    extrusion.scaleVals[tailconeNumT] = glm::dvec2(1, 1);
    extrusion.scaleVals[tailconeNumT + 1] = glm::dvec2(1, 1);
    
    double noseconeStraightLength = noseconeLength - 0.5*noseQ2zPos;

    double upperSlope = ((noseconeTipHeight-fuselageHeight)*0.5+noseconeOffset)/(noseconeStraightLength);

    //Gets control points of bezier curve
    Q2 = glm::dvec2(noseQ2zPos+noseconeStraightLength, noseQ2zPos*upperSlope+noseconeTipHeight*0.5+noseconeOffset*0.5);
    Q1 = glm::dvec2(noseconeStraightLength, -noseconeTipHeight*0.5+noseconeOffset);
    Q3 = glm::dvec2(noseconeStraightLength, noseconeTipHeight*0.5+noseconeOffset);

    for(int i = 0; i < noseconeNumT; i++){
        //Multiply by 0.5 to get tValue as loop simultaneously goes from 0 to 0.5 and 1 to 0.5
        double tVal = (double)i/noseconeNumT*0.5;

        glm::dvec2 lowerPoint = (1-tVal)*(1-tVal)*Q1+2.0*(1-tVal)*tVal*Q2+tVal*tVal*Q3;
        glm::dvec2 upperPoint = tVal*tVal*Q1+2.0*(1-tVal)*tVal*Q2+(1-tVal)*(1-tVal)*Q3;

        extrusion.zSampleVals[tailconeNumT + 2 + i] = fuselageLength + upperPoint[0];
        extrusion.posVals[tailconeNumT + 2 + i] = glm::dvec2(0, (upperPoint[1]+lowerPoint[1])/2);
        extrusion.scaleVals[tailconeNumT + 2 + i] = glm::dvec2((upperPoint[1]-lowerPoint[1])/fuselageHeight, (upperPoint[1]-lowerPoint[1])/fuselageHeight);

    }

    extrusion.translation = glm::dvec3(0.0, 0.0, 0.0);

    extrusion.pivotPoint = {0.0, 0.0, 0.0};
    extrusion.rotation = glm::dquat(glm::dvec3(0.5*M_PI, 0.0, -0.5*M_PI));

    return extrusion;
}

profile wingProfile(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double wingAirfoilMaxCamber = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingAirfoilMaxCamber") - paramNames.begin())];
    double wingAirfoilMaxCamberPos = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingAirfoilMaxCamberPos") - paramNames.begin())];
    double wingAirfoilMaxThickness = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingAirfoilMaxThickness") - paramNames.begin())];
    double wingRootChord = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingRootChord") - paramNames.begin())];


    vector<glm::dvec2> airfoilPoints = generateNACAAirfoil(wingAirfoilMaxCamber, wingAirfoilMaxCamberPos, wingAirfoilMaxThickness, wingRootChord, 0.02, meshRes);

    profile airfoilProfile(airfoilPoints);

    return airfoilProfile;
}

extrusionData extrudeLeftWing(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double wingScale = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingScale") - paramNames.begin())];
    double wingLength = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingLength") - paramNames.begin())];
    double wingSweep = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingSweep") - paramNames.begin())];

    double wingXPos = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingXPos") - paramNames.begin())];


    extrusionData extrusion;

    extrusion.zSampleVals = {0.0, -wingLength};
    extrusion.scaleVals = {glm::dvec2(1, 1), glm::dvec2(wingScale, wingScale)};
    extrusion.posVals = {glm::dvec2(0, 0), glm::dvec2(wingSweep, 0)};


    extrusion.pivotPoint = glm::dvec3(0.0, 0.0, 0.0);
    extrusion.rotation = glm::dquat(glm::dvec3(0.0, 0.5*M_PI, 0.0));
    extrusion.translation = glm::dvec3(0.0, 0.0, wingXPos);

    return extrusion;
}


extrusionData extrudeRightWing(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double wingScale = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingScale") - paramNames.begin())];
    double wingLength = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingLength") - paramNames.begin())];
    double wingSweep = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingSweep") - paramNames.begin())];

    double wingXPos = paramVals[(int)(find(paramNames.begin(), 
        paramNames.end(), "wingXPos") - paramNames.begin())];


    extrusionData extrusion;

    extrusion.zSampleVals = {0.0, wingLength};
    extrusion.scaleVals = {glm::dvec2(1, 1), glm::dvec2(wingScale, wingScale)};
    extrusion.posVals = {glm::dvec2(0, 0), glm::dvec2(wingSweep, 0)};


    extrusion.pivotPoint = glm::dvec3(0.0, 0.0, 0.0);
    extrusion.rotation = glm::dquat(glm::dvec3(0.0, 0.5*M_PI, 0.0));
    extrusion.translation = glm::dvec3(0.0, 0.0, wingXPos);

    return extrusion;
}