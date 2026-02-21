#include "MULEplane.h"

void calcDerivedParams(vector<string>& paramNames, vector<double>& paramVals, 
    const vector<dataTable>& discreteTables, vector<int> discreteVals, double& wingRootChord, 
    double& wingLength, double& wingScale, double& tailHorizArea){


    double fuselageHeight = getParam("fuselageHeight", paramVals, paramNames);
    double fuselageLength = getParam("fuselageLength", paramVals, paramNames);
    double fuselageParam1 = getParam("fuselageParam1", paramVals, paramNames);
    double fuselageParam2 = getParam("fuselageParam2", paramVals, paramNames);
    wingRootChord = getParam("wingRootChord", paramVals, paramNames);
    double wingAirfoilMaxThickness = getParam("wingAirfoilMaxThickness", paramVals, paramNames);
    double wingHorizontalPosition = getParam("wingHorizontalPosition", paramVals, paramNames);
    double wingVerticalPosition = getParam("wingVerticalPosition", paramVals, paramNames);
    wingLength = getParam("wingLength", paramVals, paramNames);
    double wingSweep = getParam("wingSweep", paramVals, paramNames);
    wingScale = getParam("wingScale", paramVals, paramNames);
    double elevatorAirfoilMaxThickness = getParam("elevatorAirfoilMaxThickness", paramVals, paramNames);
    double empennageLength = getParam("empennageLength", paramVals, paramNames);
    double tailconeLength = getParam("tailconeLength", paramVals, paramNames);
    double elevatorChord = getParam("elevatorChord", paramVals, paramNames);

    double wingXPos = (fuselageLength-wingRootChord)*wingHorizontalPosition+wingRootChord*0.5;
    double wingZPos = (wingVerticalPosition-0.5)*(fuselageHeight-wingAirfoilMaxThickness*0.01*wingRootChord);

    
    double motorWidth = getParam("Diameter", discreteTables[0].rows[discreteVals[0]].second, discreteTables[0].colNames)*0.001;
    double motorMass = getParam("Mass", discreteTables[0].rows[discreteVals[0]].second, discreteTables[0].colNames);
    double motorThrust = getParam("Thrust", discreteTables[0].rows[discreteVals[0]].second, discreteTables[0].colNames)*0.001*G_CONSTANT;
    double motorRPM = getParam("RPM", discreteTables[0].rows[discreteVals[0]].second, discreteTables[0].colNames);
    double horizontalStabiliserWidth = getParam("horizontalStabiliserWidth", paramVals, paramNames);
    double horizontalStabiliserChord = getParam("horizontalStabiliserChord", paramVals, paramNames);
    double horizontalStabiliserAirfoilMaxCamber = getParam("horizontalStabiliserAirfoilMaxCamber", paramVals, paramNames);
    double horizontalStabiliserAirfoilMaxThickness = getParam("horizontalStabiliserAirfoilMaxThickness", paramVals, paramNames);
    double propellerWidth = getParam("propDiameter", discreteTables[0].rows[discreteVals[0]].second, discreteTables[0].colNames)*0.025 + 0.1;
    double propPitch = getParam("Diameter", discreteTables[0].rows[discreteVals[0]].second, discreteTables[0].colNames)*0.025;
    double motorLength = getParam("Length", discreteTables[0].rows[discreteVals[0]].second, discreteTables[0].colNames)*0.001;

    //Finds position of wing in y direction, as in the distance from the centre of the fuselage
    //Inverts Bezier curve to get wing position
    //See https://www.desmos.com/calculator/wplkmlrji6
    double tVal = cos(acos(2.0*wingVerticalPosition-1.0)/3.0-2.0*M_PI/3.0)+0.5;

    //Substitutes t value back into bezier curve to get wing position
    double wingYPos = tVal * 3.0 * (1.0-tVal)*(1.0-tVal) * fuselageParam1 + 3.0 * (1.0-tVal) * tVal*tVal * fuselageParam2 - 0.05;
    

    //Finds motor position relative to base of wing
    double motorPodYPos = 0.5*propellerWidth + horizontalStabiliserWidth*(wingLength-0.5*propellerWidth);
    //Scaling of wing between root and motor pod
    double wingBaseScale = 1.0-(1.0-wingScale)*((motorPodYPos - motorWidth*0.5)/wingLength);

    double wingBaseSweep = (motorPodYPos - motorWidth*0.5)/wingLength*wingSweep;

    //X position of start and end of motorPod
    double motorPodLength = wingRootChord*wingBaseScale;
    double motorPodXPos = wingBaseSweep + 0.5*motorPodLength;//+ 0.5*wingRootChord*(1-wingBaseScale);

    double batteryMass = getParam("Weight", discreteTables[1].rows[discreteVals[1]].second, discreteTables[1].colNames)*0.001;

    double empennageBoomLength = wingXPos - wingBaseSweep - 0.5*motorPodLength + empennageLength + tailconeLength;

    double fullHorizontalStabiliserWidth = propellerWidth + 2.0*wingYPos+2.0*horizontalStabiliserWidth*(wingLength-0.5*propellerWidth);

    //double flapSweep = wingSweep - 0.5*wingRootChord*wingScale*(1-wingScale);
    //Position of elevator pivot point in x direction
    double elevatorRadius = 1.1019*(elevatorAirfoilMaxThickness/100)*(elevatorAirfoilMaxThickness/100)*elevatorChord;

    double boomRodThickness = horizontalStabiliserAirfoilMaxThickness*0.01*horizontalStabiliserChord + 
        0.5*horizontalStabiliserAirfoilMaxCamber*horizontalStabiliserChord*0.1;

    vector<double> newParamVals = {wingXPos, wingYPos, wingZPos, motorWidth, motorLength, motorPodXPos, motorPodYPos,
         motorPodLength, motorMass, motorThrust, motorRPM, propPitch, empennageBoomLength, 
         fullHorizontalStabiliserWidth, boomRodThickness, elevatorRadius, batteryMass};
    vector<string> newParamNames = {"wingXPos", "wingYPos", "wingZPos", "motorWidth", "motorLength",
         "motorPodXPos", "motorPodYPos", "motorPodLength", "motorMass", "motorThrust", "motorRPM", 
         "propPitch", "empennageBoomLength", "fullHorizontalStabiliserWidth", "boomRodThickness",
         "elevatorRadius", "batteryMass"};


    //Adds new parameters to list
    paramVals.insert(paramVals.end(), newParamVals.begin(), newParamVals.end());
    paramNames.insert(paramNames.end(), newParamNames.begin(), newParamNames.end());


    //Calculates global variables
    tailHorizArea = fullHorizontalStabiliserWidth*horizontalStabiliserChord;
}

double rateDesign(array<double, 3> aircraftConfig, double velocity, glm::dvec3 aeroForces,
    double oscillationFreq, double dampingCoeff, double dMdAlpha, double mass, vector<string> fullParamNames, vector<double> fullParamVals){
    
    return dampingCoeff*(1.0/oscillationFreq)*(-1.0/aeroForces[0]);
};

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
    double batteryMass = getParam("batteryMass", paramVals, paramNames);
    double batteryPos = getParam("batteryPos", paramVals, paramNames);


    double tailconeTipHeight = tailconeTipScale*fuselageHeight;
    double noseconeTipHeight = noseconeTipScale*fuselageHeight;
    //Finds the length of the uncurved section of the nosecone
    double noseQ2zPos = (noseconeTipHeight*noseconeLength)/(fuselageHeight-0.5*noseconeTipHeight);
    double tailQ2zPos = (tailconeTipHeight*tailconeLength)/(fuselageHeight-0.5*tailconeTipHeight);

    int tailconeNumT = ceil(0.5*meshRes*tailQ2zPos);
    int noseconeNumT = ceil(0.5*meshRes*noseQ2zPos);

    vector<double> zSampleVals;
    vector<glm::dvec2> posVals, scaleVals;
    zSampleVals.resize(noseconeNumT + 2 + tailconeNumT);
    posVals.resize(noseconeNumT + 2 + tailconeNumT);
    scaleVals.resize(noseconeNumT + 2 + tailconeNumT);


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
        zSampleVals[tailconeNumT - i - 1] = upperPoint[0];
        posVals[tailconeNumT - i - 1] = glm::dvec2(0, (upperPoint[1]+lowerPoint[1])/2);
        scaleVals[tailconeNumT - i - 1] = glm::dvec2((upperPoint[1]-lowerPoint[1])/fuselageHeight, (upperPoint[1]-lowerPoint[1])/fuselageHeight);

    }

    //Straight section of fuselage
    zSampleVals[tailconeNumT] = 0;
    zSampleVals[tailconeNumT + 1] = fuselageLength;
    posVals[tailconeNumT] = glm::dvec2(0, 0);
    posVals[tailconeNumT + 1] = glm::dvec2(0, 0);
    scaleVals[tailconeNumT] = glm::dvec2(1, 1);
    scaleVals[tailconeNumT + 1] = glm::dvec2(1, 1);
    
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

        zSampleVals[tailconeNumT + 2 + i] = fuselageLength + upperPoint[0];
        posVals[tailconeNumT + 2 + i] = glm::dvec2(0, (upperPoint[1]+lowerPoint[1])/2);
        scaleVals[tailconeNumT + 2 + i] = glm::dvec2((upperPoint[1]-lowerPoint[1])/fuselageHeight, (upperPoint[1]-lowerPoint[1])/fuselageHeight);

    }

    glm::dvec3 translation(0.0, 0.0, 0.0);

    glm::dvec3 pivotPoint(0.0, 0.0, 0.0);
    glm::dquat rotation(glm::dvec3(0.5*M_PI, 0.0, 0.5*M_PI));

    //Adds battery point mass
    double pointMass = batteryMass;
    glm::dvec3 massLocation(0.0, 0.0, batteryPos*fuselageLength);

    extrusionData extrusion(zSampleVals, posVals, scaleVals, rotation, translation, 
        pivotPoint, pointMass, massLocation);

    return extrusion;
}

profile wingProfile(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double wingAirfoilMaxCamber = getParam("wingAirfoilMaxCamber", paramVals, paramNames);
    double wingAirfoilMaxCamberPos = getParam("wingAirfoilMaxCamberPos", paramVals, paramNames);
    double wingAirfoilMaxThickness = getParam("wingAirfoilMaxThickness", paramVals, paramNames);
    double wingRootChord = getParam("wingRootChord", paramVals, paramNames);

    double insetThickness = 0.005;

    vector<glm::dvec2> airfoilPoints = generateNACAAirfoil(wingAirfoilMaxCamber, wingAirfoilMaxCamberPos, wingAirfoilMaxThickness, wingRootChord, meshRes);

    profile airfoilProfile(airfoilPoints, insetThickness);

    //airfoilProfile.plot(500, 500);

    return airfoilProfile;
}

extrusionData extrudeLeftWing(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double wingScale = getParam("wingScale", paramVals, paramNames);
    double wingLength = getParam("wingLength", paramVals, paramNames);
    double wingSweep = getParam("wingSweep", paramVals, paramNames);
    double wingXPos = getParam("wingXPos", paramVals, paramNames);
    double wingYPos = getParam("wingYPos", paramVals, paramNames);
    double wingZPos = getParam("wingZPos", paramVals, paramNames);
    

    vector<double> zSampleVals = {0.0, wingLength};
    vector<glm::dvec2> scaleVals = {glm::dvec2(1.0, 1.0), wingScale*glm::dvec2(1.0, 1.0)};
    vector<glm::dvec2> posVals{glm::dvec2(0.0, 0.0), glm::dvec2(wingSweep, 0.0)};

    glm::dvec3 pivotPoint(0.0, 0.0, 0.0);
    glm::dquat rotation(glm::dvec3(0.0, 0.5*M_PI, 0.0));
    glm::dvec3 translation(wingYPos, wingZPos, wingXPos);


    extrusionData extrusion(zSampleVals, posVals, scaleVals, rotation, translation, pivotPoint);

    return extrusion;
}

extrusionData extrudeRightWing(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double wingScale = getParam("wingScale", paramVals, paramNames);
    double wingLength = getParam("wingLength", paramVals, paramNames);
    double wingSweep = getParam("wingSweep", paramVals, paramNames);
    double wingXPos = getParam("wingXPos", paramVals, paramNames);
    double wingYPos = getParam("wingYPos", paramVals, paramNames);
    double wingZPos = getParam("wingZPos", paramVals, paramNames);


    vector<double> zSampleVals = {0.0, -wingLength};
    vector<glm::dvec2> scaleVals = {glm::dvec2(1.0, 1.0), wingScale*glm::dvec2(1.0, 1.0)};
    vector<glm::dvec2> posVals = {glm::dvec2(0.0, 0.0), glm::dvec2(wingSweep, 0.0)};


    glm::dvec3 pivotPoint = glm::dvec3(0.0, 0.0, 0.0);
    glm::dquat rotation = glm::dquat(glm::dvec3(0.0, 0.5*M_PI, 0.0));
    glm::dvec3 translation = glm::dvec3(-wingYPos, wingZPos, wingXPos);

    extrusionData extrusion(zSampleVals, posVals, scaleVals, rotation, translation, pivotPoint);

    return extrusion;
}

profile motorPodProfile(vector<string> paramNames, vector<double> paramVals, double meshRes){

    double motorWidth = getParam("motorWidth", paramVals, paramNames);

    double insetThickness = 0.01;

    double podWidth = 2*insetThickness + motorWidth;

    //Gets number of points and then multiplies by 2 to account for small size of motor
    int halfNumPoints = ceil(podWidth*M_PI*0.5*meshRes);
    vector<glm::dvec2> profilePoints(halfNumPoints*2);

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

extrusionData extrudeRightMotorPod(vector<string> paramNames, vector<double> paramVals, double meshRes){

    double motorPodLength = getParam("motorPodLength", paramVals, paramNames);
    double motorLength = getParam("motorLength", paramVals, paramNames);
    double motorPodXPos = getParam("motorPodXPos", paramVals, paramNames);
    double motorPodYPos = getParam("motorPodYPos", paramVals, paramNames);
    double motorThrust = getParam("motorThrust", paramVals, paramNames);
    double motorRPM = getParam("motorRPM", paramVals, paramNames);
    double motorMass = getParam("motorMass", paramVals, paramNames);
    double propPitch = getParam("propPitch", paramVals, paramNames);


    vector<double> zSampleVals = {0.0, -motorPodLength};
    vector<glm::dvec2> scaleVals = {glm::dvec2(1.0, 1.0), glm::dvec2(1.0, 1.0)};
    vector<glm::dvec2> posVals = {glm::dvec2(0.0, 0.0), glm::dvec2(0.0, 0.0)};


    glm::dvec3 pivotPoint = glm::dvec3(0.0, 0.0, 0.0);
    glm::dquat rotation = glm::dquat(glm::dvec3(0.0, 0.5*M_PI, 0.0));
    glm::dvec3 translation = glm::dvec3(motorPodXPos, 0.0, -motorPodYPos);

    motorData motor(glm::dvec3(0.0, 0.0, 1.0), glm::dvec3(0, 0.0, -motorPodLength-0.05), 
        motorThrust, motorRPM, propPitch);

    extrusionData extrusion(zSampleVals, posVals, scaleVals, rotation, translation, 
        pivotPoint, motorMass, glm::dvec3(0.0, 0.0, -motorPodLength + 0.5*motorLength), motor);

    return extrusion;
}

extrusionData extrudeLeftMotorPod(vector<string> paramNames, vector<double> paramVals, double meshRes){

    double motorPodLength = getParam("motorPodLength", paramVals, paramNames);
    double motorPodXPos = getParam("motorPodXPos", paramVals, paramNames);
    double motorPodYPos = getParam("motorPodYPos", paramVals, paramNames);
    double motorThrust = getParam("motorThrust", paramVals, paramNames);
    double motorRPM = getParam("motorRPM", paramVals, paramNames);
    double motorLength = getParam("motorLength", paramVals, paramNames);
    double motorMass = getParam("motorMass", paramVals, paramNames);
    double propPitch = getParam("propPitch", paramVals, paramNames);


    vector<double> zSampleVals = {0.0, -motorPodLength};
    vector<glm::dvec2> scaleVals = {glm::dvec2(1.0, 1.0), glm::dvec2(1.0, 1.0)};
    vector<glm::dvec2> posVals = {glm::dvec2(0.0, 0.0), glm::dvec2(0.0, 0.0)};


    glm::dvec3 pivotPoint = glm::dvec3(0.0, 0.0, 0.0);
    glm::dquat rotation = glm::dquat(glm::dvec3(0.0, 0.5*M_PI, 0.0));
    glm::dvec3 translation = glm::dvec3(motorPodXPos, 0.0, motorPodYPos);

    motorData motor(glm::dvec3(0.0, 0.0, 1.0), glm::dvec3(0, 0.0, -motorPodLength-0.05), 
        motorThrust, motorRPM, propPitch);

    extrusionData extrusion(zSampleVals, posVals, scaleVals, rotation, translation, 
        pivotPoint, motorMass, glm::dvec3(0.0, 0.0, -motorPodLength + 0.5*motorLength), motor);

    return extrusion;
}

profile empennageBoomProfile(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double boomDiameter = 0.04;

    int nPoints = ceil(boomDiameter*M_PI*meshRes);

    vector<glm::dvec2> points(nPoints);

    for(int i = 0; i < nPoints; i ++){
        double angle = 2.0*M_PI*i/(nPoints-1.0);
        points[i] = 0.5*boomDiameter*glm::dvec2(cos(angle), sin(angle));
    }

    profile boomProfile(points, 0.02);

    //boomProfile.plot(500, 500);

    return boomProfile;
}

extrusionData extrudeEmpennageBoom(vector<string> paramNames, vector<double> paramVals, double meshRes){

    double horizontalStabiliserChord = getParam("horizontalStabiliserChord", paramVals, paramNames);
    double boomLength = getParam("empennageBoomLength", paramVals, paramNames);
    double boomRodThickness = getParam("boomRodThickness", paramVals, paramNames);


    double boomDiameter = 0.04;
    double rodThickness = max(boomDiameter, boomRodThickness);

    vector<double> zSampleVals = {0.0, boomLength, boomLength+0.01, boomLength+horizontalStabiliserChord};
    vector<glm::dvec2> scaleVals = {glm::dvec2(1.0, 1.0), glm::dvec2(1.0, 1.0),
        rodThickness/boomDiameter*glm::dvec2(1.0, 1.0), rodThickness/boomDiameter*glm::dvec2(1.0, 1.0)};
    vector<glm::dvec2> posVals = {glm::dvec2(0.0, 0.0), glm::dvec2(0.0, 0.0), glm::dvec2(0.0, 0.0), glm::dvec2(0.0, 0.0)};

    glm::dvec3 pivotPoint = glm::dvec3(0.0, 0.0, 0.0);
    glm::dquat rotation = glm::dquat(glm::dvec3(0.0, 0.0, 0.0));
    glm::dvec3 translation = glm::dvec3(0.0, 0.0, 0.0);

    extrusionData extrusion(zSampleVals, posVals, scaleVals, rotation, translation, pivotPoint);

    return extrusion;
}

profile horizontalStabiliserProfile(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double horizontalStabiliserAirfoilMaxCamber = getParam("horizontalStabiliserAirfoilMaxCamber", paramVals, paramNames);
    double horizontalStabiliserAirfoilMaxCamberPos = getParam("horizontalStabiliserAirfoilMaxCamberPos", paramVals, paramNames);
    double horizontalStabiliserAirfoilMaxThickness = getParam("horizontalStabiliserAirfoilMaxThickness", paramVals, paramNames);
    double horizontalStabiliserChord = getParam("horizontalStabiliserChord", paramVals, paramNames);
    double elevatorRadius = getParam("elevatorRadius", paramVals, paramNames);

    double insetThickness = 0.005;

    vector<glm::dvec2> airfoilPoints = generateNACAAirfoil(horizontalStabiliserAirfoilMaxCamber, 
        horizontalStabiliserAirfoilMaxCamberPos, horizontalStabiliserAirfoilMaxThickness, 
        horizontalStabiliserChord, elevatorRadius, meshRes);

    profile airfoilProfile(airfoilPoints, insetThickness);

    //airfoilProfile.plot(500, 500);

    return airfoilProfile;
}

extrusionData extrudeHorizontalStabiliser(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double stabiliserWidth = getParam("fullHorizontalStabiliserWidth", paramVals, paramNames);
    double empennageBoomLength = getParam("empennageBoomLength", paramVals, paramNames);
    double stabiliserChord = getParam("horizontalStabiliserChord", paramVals, paramNames);
    double boomRodThickness = getParam("boomRodThickness", paramVals, paramNames);

    double xPos = empennageBoomLength + 0.5*stabiliserChord;

    vector<double> zSampleVals = {0.0, stabiliserWidth - boomRodThickness};
    vector<glm::dvec2> scaleVals = {glm::dvec2(1.0, 1.0), glm::dvec2(1.0, 1.0)};
    vector<glm::dvec2> posVals = {glm::dvec2(0.0, 0.0), glm::dvec2(0.0, 0.0)};


    glm::dvec3 pivotPoint = glm::dvec3(0.0, 0.0, 0.0);
    glm::dquat rotation = glm::dquat(glm::dvec3(0.0, -0.5*M_PI, 0.0));
    glm::dvec3 translation = glm::dvec3(-0.5*boomRodThickness, 0.0, xPos);

    extrusionData extrusion(zSampleVals, posVals, scaleVals, rotation, translation, pivotPoint, true);

    return extrusion;
}


profile elevatorProfile(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double elevatorAirfoilMaxThickness = getParam("elevatorAirfoilMaxThickness", paramVals, paramNames);
    double elevatorChord = getParam("elevatorChord", paramVals, paramNames);

    double insetThickness = 0.01;


    vector<glm::dvec2> airfoilPoints = generateNACAAirfoil(0.0, 5, elevatorAirfoilMaxThickness,
        elevatorChord, meshRes);

    profile airfoilProfile(airfoilPoints, insetThickness);

    //airfoilProfile.plot(500, 500);

    return airfoilProfile;
}


extrusionData extrudeElevator(vector<string> paramNames, vector<double> paramVals, double meshRes){
    double elevatorWidth = getParam("fullHorizontalStabiliserWidth", paramVals, paramNames);
    double elevatorRadius = getParam("elevatorRadius", paramVals, paramNames);
    double elevatorChord = getParam("elevatorChord", paramVals, paramNames);
    double stabiliserChord = getParam("horizontalStabiliserChord", paramVals, paramNames);
    double boomRodThickness = getParam("boomRodThickness", paramVals, paramNames);

    double elevatorPivot = elevatorRadius - 0.5*elevatorChord;

    double xPos = 0.5*stabiliserChord - 0.5*elevatorRadius;

    vector<double> zSampleVals = {0.0, elevatorWidth - boomRodThickness};
    vector<glm::dvec2> scaleVals = {glm::dvec2(1.0, 1.0), glm::dvec2(1.0, 1.0)};
    vector<glm::dvec2> posVals = {glm::dvec2(0.0, 0.0), glm::dvec2(0.0, 0.0)};

    glm::dvec3 pivotPoint(elevatorPivot, 0.0, 0.0);


    glm::dquat rotation(glm::dvec3(0.0, 0.0, 0.0));
    glm::dvec3 translation(xPos, 0.0, 0.0);
    glm::dvec3 controlAxis(0.0, 0.0, elevatorWidth);
    controlAxis /= glm::length(controlAxis);

    extrusionData extrusion(zSampleVals, posVals, scaleVals, rotation, translation, 
        pivotPoint, controlAxis, true);

    return extrusion;
}