#include "MULEplane.h"

void calcDerivedParams(vector<string>& paramNames, vector<double>& paramVals, 
    const vector<dataTable>& discreteTables, vector<int> discreteVals){

    double fuselageLength = getParam("fuselageLength", paramVals, paramNames);
    double wingRootChord = getParam("wingRootChord", paramVals, paramNames);
    double wingHorizontalPosition = getParam("wingHorizontalPosition", paramVals, paramNames);
    double wingSweep = getParam("wingSweep", paramVals, paramNames);
    double wingScale = getParam("wingScale", paramVals, paramNames);
    double elevatorMaxThickness = getParam("elevatorMaxThickness", paramVals, paramNames);
    double elevatorChord = getParam("elevatorChord", paramVals, paramNames);

    double wingXPos = (fuselageLength-wingRootChord)*wingHorizontalPosition+wingRootChord*0.5;

    double motorWidth = getParam("Diameter", discreteTables[0].rows[discreteVals[0]].second, discreteTables[0].colNames);

    double motorMass = getParam("Weight", discreteTables[0].rows[discreteVals[0]].second, discreteTables[0].colNames);

    double elevatorSweep = wingSweep- 0.5*wingRootChord*wingScale*(1-wingScale);


    //Position of elevator pivot point in x direction
    double elevatorRadius = 1.1019*(elevatorMaxThickness/100)*(elevatorMaxThickness/100)*elevatorChord;


    vector<double> newParamVals = {wingXPos, motorWidth, motorMass, elevatorSweep,
                                   elevatorRadius};
    vector<string> newParamNames = {"wingXPos", "motorWidth", "motorMass", "elevatorSweep",
                                   "elevatorRadius"};

    //Adds new parameters to list
    paramVals.insert(paramVals.end(), newParamVals.begin(), newParamVals.end());
    paramNames.insert(paramNames.end(), newParamNames.begin(), newParamNames.end());

}

profile fuselageProfile(vector<string> paramNames, vector<double> paramVals, double meshRes){

    double fuselageHeight = getParam("fuselageHeight", paramVals, paramNames);
    double fuselageParam1 = getParam("fuselageParam1", paramVals, paramNames);
    double fuselageParam2 = getParam("fuselageParam2", paramVals, paramNames);

    //double plasticThickness = 0.05;

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
        double tVal = (double)i/(numPoints - 1);
        bezPoints[i] = (1 - tVal)*(1 - tVal)*(1 - tVal) * p1 + tVal * 3 * (1-tVal)*(1-tVal) * p2
                     + (3 * (1-tVal) * tVal*tVal) * p3 + tVal*tVal*tVal * p4;
    }

    //Copies points to opposite side of profile
    for(int i = 1; i < numPoints - 1; i++){
        bezPoints[2*numPoints - 2 - i] = glm::dvec2(-1.0*bezPoints[i][0], bezPoints[i][1]);
    }


    profile outputProfile(bezPoints);//, plasticThickness);

    //outputProfile.plot(500, 500);

    return outputProfile;
}


extrusionData extrudeFuselage(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double fuselageLength = getParam("fuselageLength", paramVals, paramNames);
    double fuselageHeight = getParam("fuselageHeight", paramVals, paramNames);
    double noseconeLength = getParam("noseconeLength", paramVals, paramNames);
    double noseconeTipScale = getParam("noseconeTipScale", paramVals, paramNames);
    double noseconeOffset = getParam("noseconeOffset", paramVals, paramNames);
    double tailconeLength = getParam("tailconeLength", paramVals, paramNames);
    double tailconeTipScale = getParam("tailconeTipScale", paramVals, paramNames);

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
    extrusion.rotation = glm::dquat(glm::dvec3(0.5*M_PI, 0.0, 0.5*M_PI));
    extrusion.isControl = false;


    return extrusion;
}

profile wingProfile(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double wingAirfoilMaxCamber = getParam("wingAirfoilMaxCamber", paramVals, paramNames);
    double wingAirfoilMaxCamberPos = getParam("wingAirfoilMaxCamberPos", paramVals, paramNames);
    double wingAirfoilMaxThickness = getParam("wingAirfoilMaxThickness", paramVals, paramNames);
    double wingRootChord = getParam("wingRootChord", paramVals, paramNames);
    double elevatorRadius = getParam("elevatorRadius", paramVals, paramNames);

    double insetThickness = 0.005;

    vector<glm::dvec2> airfoilPoints = generateNACAAirfoil(wingAirfoilMaxCamber, wingAirfoilMaxCamberPos, wingAirfoilMaxThickness, wingRootChord, elevatorRadius, meshRes);

    profile airfoilProfile(airfoilPoints, insetThickness);

    //airfoilProfile.plot(500, 500);

    return airfoilProfile;
}

extrusionData extrudeLeftWing(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double wingScale = getParam("wingScale", paramVals, paramNames);
    double wingLength = getParam("wingLength", paramVals, paramNames);
    double wingSweep = getParam("wingSweep", paramVals, paramNames);
    double wingXPos = getParam("wingXPos", paramVals, paramNames);

    extrusionData extrusion;

    extrusion.zSampleVals = {0.0, wingLength};
    extrusion.scaleVals = {glm::dvec2(1, 1), glm::dvec2(wingScale, wingScale)};
    extrusion.posVals = {glm::dvec2(0, 0), glm::dvec2(wingSweep, 0)};


    extrusion.pivotPoint = glm::dvec3(0.0, 0.0, 0.0);
    extrusion.rotation = glm::dquat(glm::dvec3(0.0, 0.5*M_PI, 0.0));
    extrusion.translation = glm::dvec3(0.0, 0.0, wingXPos);
    extrusion.isControl = false;

    return extrusion;
}


extrusionData extrudeRightWing(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double wingScale = getParam("wingScale", paramVals, paramNames);
    double wingLength = getParam("wingLength", paramVals, paramNames);
    double wingSweep = getParam("wingSweep", paramVals, paramNames);
    double wingXPos = getParam("wingXPos", paramVals, paramNames);


    extrusionData extrusion;

    extrusion.zSampleVals = {0.0, -wingLength};
    extrusion.scaleVals = {glm::dvec2(1, 1), glm::dvec2(wingScale, wingScale)};
    extrusion.posVals = {glm::dvec2(0, 0), glm::dvec2(wingSweep, 0)};


    extrusion.pivotPoint = glm::dvec3(0.0, 0.0, 0.0);
    extrusion.rotation = glm::dquat(glm::dvec3(0.0, 0.5*M_PI, 0.0));
    extrusion.translation = glm::dvec3(0.0, 0.0, wingXPos);
    extrusion.isControl = false;

    return extrusion;
}


profile motorPodProfile(vector<string> paramNames, vector<double> paramVals, double meshRes){

    double motorWidth = getParam("motorWidth", paramVals, paramNames);

    double insetThickness = 0.01;

    double podWidth = 2*insetThickness + motorWidth;

    //Gets number of points and then multiplies by 2 to account for small size of motor
    int halfNumPoints = ceil(podWidth*M_PI*0.5*meshRes);
    vector<glm::dvec2> profilePoints;
    profilePoints.resize(halfNumPoints*2);

    for(int i = 0; i < halfNumPoints; i++){
        double angle = i*M_PI/(halfNumPoints - 1);

        profilePoints[i][0] = -0.5*podWidth*cos(angle);
        profilePoints[i][1] = 0.5*podWidth*sin(angle) + 0.25*podWidth;

        profilePoints[halfNumPoints + i][0] = 0.5*podWidth*cos(angle);
        profilePoints[halfNumPoints + i][1] = -0.5*podWidth*sin(angle) - 0.25*podWidth;
    }

    profile motorProfile(profilePoints, insetThickness);

    //motorProfile.plot(500, 500);

    return motorProfile;
    
}

profile elevatorProfile(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double elevatorChord = getParam("elevatorChord", paramVals, paramNames);
    double elevatorMaxThickness = getParam("elevatorMaxThickness", paramVals, paramNames);
    
    double insetThickness = 0.01;

    vector<glm::dvec2> airfoilPoints = generateNACAAirfoil(0.0, 0.0, elevatorMaxThickness, elevatorChord, meshRes);

    profile airfoilProfile(airfoilPoints, insetThickness);

    //airfoilProfile.plot(500, 500);

    return airfoilProfile;
}



extrusionData extrudeRightElevator(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double wingRootChord = getParam("wingRootChord", paramVals, paramNames);
    double elevatorChord = getParam("wingRootChord", paramVals, paramNames);
    double wingLength = getParam("wingLength", paramVals, paramNames);
    double elevatorSweep = getParam("elevatorSweep", paramVals, paramNames);
    double elevatorRadius = getParam("elevatorRadius", paramVals, paramNames);
    double elevatorPivot = -elevatorRadius + 0.5*elevatorChord;

    extrusionData extrusion;

    extrusion.zSampleVals = {0.0, -wingLength};
    extrusion.scaleVals = {glm::dvec2(1.0, 1.0), glm::dvec2(1.0, 1.0)};
    extrusion.posVals = {glm::dvec2(0.0, 0.0), glm::dvec2(0.2, 0.0)};

    extrusion.pivotPoint = glm::dvec3(elevatorPivot, 0.0, 0.0);


    extrusion.rotation = glm::dquat(glm::dvec3(0.0, 0.0, 0.0));
    extrusion.translation = glm::dvec3(0.5*wingRootChord, 0.5, 0.0);
    extrusion.isControl = true;
    //extrusion.controlAxis = glm::dvec3(elevatorSweep, 0.0, wingLength);
    extrusion.controlAxis = glm::dvec3(elevatorSweep, 0.0, -wingLength);
    extrusion.controlAxis /= glm::length(extrusion.controlAxis);


    return extrusion;
}