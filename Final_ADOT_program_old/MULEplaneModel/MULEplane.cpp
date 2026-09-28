#include "MULEplane.h"

aircraft constructAircraft(){

    // Creates vector of profile functions
    vector<function<profile(vector<string>, vector<double>, double)>> profileFunctions = 
        {fuselageProfile, wingProfile, motorPodProfile, empennageBoomProfile, 
            horizontalStabiliserProfile, elevatorProfile};
    
    // Gets maximum and minimun values of each parameter
    dataTable paramTable = readCSV("MULEplaneModel/paramRanges.csv");
    int nParams = paramTable.rows.size();
    // Gets tables of discrete values
    dataTable motorTable = readCSV("MULEplaneModel/motorSelection.csv");
    dataTable batteryTable = readCSV("MULEplaneModel/batterySelection.csv");

    // Gets names and ranges of parameters
    vector<string> paramNames(nParams);
    vector<glm::dvec2> paramRanges(nParams);
    for(int i = 0; i < nParams; i++){
        paramNames[i] = paramTable.rows[i].first;
        paramRanges[i][0] = paramTable.rows[i].second[0];
        paramRanges[i][1] = paramTable.rows[i].second[1];
    }
    vector<dataTable> discreteTables = {motorTable, batteryTable};


    // Describes positions at which simulations will be conducted
    vector<vector<double>> positionValues;
    double testVelocity = 40.0;
    // Gets values based on pitch and elevator values
    double minPitch = -M_PI/18.0;
    double maxPitch = M_PI/6.0;
    int nPitch = 4;

    double minElevator = -M_PI/6.0;
    double maxElevator = M_PI/6.0;
    int nElevator = 4;
    // Fills array
    for(int e = 0; e < nElevator; e++){  
        for(int p = 0; p < nPitch; p++){
            double pitch = minPitch + p * (maxPitch - minPitch)/(nPitch-1);
            double velX = -1.0*testVelocity*cos(pitch);
            double velZ = testVelocity*sin(pitch);
            double gravX = -sin(pitch);
            double gravZ = -cos(pitch);
            double elevatorAngle = minElevator + e * (maxElevator - minElevator)/(nElevator-1);

            positionValues[e*nPitch+p] = {velX, 0.0, velZ, gravX, 0.0, gravZ, elevatorAngle};
        }
    }
    
    aircraft MULEaircraft(paramNames, paramRanges, discreteTables, calcDerivedParams, profileFunctions, 
        positionValues, rateDesign, 0.0005);




    MULEaircraft.addPart("fuselage", 1000, extrudeFuselage, 0);
    MULEaircraft.addPart("rightWing", "fuselage", 1000, extrudeRightWing, 1);
    MULEaircraft.addPart("leftWing", "fuselage", 1000, extrudeLeftWing, 1);
    MULEaircraft.addPart("motorPodRight", "rightWing", 1000, extrudeRightMotorPod, 2);
    MULEaircraft.addPart("motorPodLeft", "leftWing", 1000, extrudeLeftMotorPod, 2);
    MULEaircraft.addPart("empennageBoomRight", "motorPodRight", 1000, extrudeEmpennageBoom, 3);
    MULEaircraft.addPart("empennageBoomLeft", "motorPodLeft", 1000, extrudeEmpennageBoom, 3);
    MULEaircraft.addPart("horizontalStabiliser", "empennageBoomRight", 1000, extrudeHorizontalStabiliser, 4);
    MULEaircraft.addPart("elevator", "horizontalStabiliser", 1000, extrudeElevator, 5);

    //MULEaircraft.plot(500, 500, paramVals, discreteVals, 50.0);

    return MULEaircraft;
}

void calcDerivedParams(vector<string>& paramNames, vector<double>& paramVals, 
    const vector<dataTable>& discreteTables, vector<int> discreteVals, vector<glm::dmat2x3>& velocityRegions){

    const double G_CONSTANT = 9.81;

    double fuselageHeight = getParam("fuselageHeight", paramVals, paramNames);
    double fuselageLength = getParam("fuselageLength", paramVals, paramNames);
    double fuselageParam1 = getParam("fuselageParam1", paramVals, paramNames);
    double fuselageParam2 = getParam("fuselageParam2", paramVals, paramNames);
    double wingRootChord = getParam("wingRootChord", paramVals, paramNames);
    double wingAirfoilMaxThickness = getParam("wingAirfoilMaxThickness", paramVals, paramNames);
    double wingHorizontalPosition = getParam("wingHorizontalPosition", paramVals, paramNames);
    double wingVerticalPosition = getParam("wingVerticalPosition", paramVals, paramNames);
    double wingLength = getParam("wingLength", paramVals, paramNames);
    double wingSweep = getParam("wingSweep", paramVals, paramNames);
    double wingScale = getParam("wingScale", paramVals, paramNames);
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

    //Calculates tail area
    double tailHorizArea = fullHorizontalStabiliserWidth*horizontalStabiliserChord;

    //double flapSweep = wingSweep - 0.5*wingRootChord*wingScale*(1-wingScale);
    //Position of elevator pivot point in x direction
    double elevatorRadius = 1.1019*(elevatorAirfoilMaxThickness/100)*(elevatorAirfoilMaxThickness/100)*elevatorChord;

    double boomRodThickness = horizontalStabiliserAirfoilMaxThickness*0.01*horizontalStabiliserChord + 
        0.5*horizontalStabiliserAirfoilMaxCamber*horizontalStabiliserChord*0.1;

    vector<double> newParamVals = {wingXPos, wingYPos, wingZPos, motorWidth, motorLength, motorPodXPos, motorPodYPos,
         motorPodLength, motorMass, motorThrust, motorRPM, propPitch, empennageBoomLength, 
         fullHorizontalStabiliserWidth, boomRodThickness, elevatorRadius, batteryMass,
         tailHorizArea};
    vector<string> newParamNames = {"wingXPos", "wingYPos", "wingZPos", "motorWidth", "motorLength",
         "motorPodXPos", "motorPodYPos", "motorPodLength", "motorMass", "motorThrust", "motorRPM", 
         "propPitch", "empennageBoomLength", "fullHorizontalStabiliserWidth", "boomRodThickness",
         "elevatorRadius", "batteryMass", "tailHorizArea"};
         

    //Adds new parameters to list
    paramVals.insert(paramVals.end(), newParamVals.begin(), newParamVals.end());
    paramNames.insert(paramNames.end(), newParamNames.begin(), newParamNames.end());

    // Outlines region just in front of horizontal stabiliser
    double tailVelRegionX = -tailconeLength - empennageLength + 0.02;
    double tailVelRegionZ = wingZPos;
    double tailVelRegionDepth = 0.05;
    double tailVelRegionHeight = horizontalStabiliserAirfoilMaxThickness*0.01*horizontalStabiliserChord;
    double tailVelRegionWidth = fullHorizontalStabiliserWidth - 0.04;
    velocityRegions = {glm::dmat2x3(tailVelRegionX, -0.5*tailVelRegionWidth, tailVelRegionZ-tailVelRegionHeight,
        tailVelRegionX+tailVelRegionDepth, 0.5*tailVelRegionWidth, tailVelRegionZ+tailVelRegionHeight)};

}

double rateDesign(vector<string> fullParamNames, vector<double> fullParamVals, 
    vector<dataTable> discreteTables, vector<int> discreteVals, vector<vector<double>> positionVariables,
    double mass, vector<glm::dvec3> COMs, vector<glm::dmat3> MOIs, vector<glm::dvec3> totalForces, 
    vector<glm::dvec3> totalTorques, vector<vector<glm::dvec3>> regionForces,
    vector<vector<glm::dvec3>> regionTorques, vector<vector<double>> regionVelMags,
    vector<vector<glm::dvec3>> POIs, vector<glm::dvec3> partDirections){

    const int G_CONSTANT = 9.81;
    const int RHO = 1.225;
    double wingRootChord = getParam("wingRootChord", fullParamVals, fullParamNames);
    double wingLength = getParam("wingLength", fullParamVals, fullParamNames);
    double wingScale = getParam("wingScale", fullParamVals, fullParamNames);
    double tailHorizArea = getParam("tailHorizArea", fullParamVals, fullParamNames);
    double motorMaxThrust = getParam("motorThrust", fullParamVals, fullParamNames);
    double motorMaxRPM = getParam("motorRPM", fullParamVals, fullParamNames);
    double motorPropPitch = getParam("propPitch", fullParamVals, fullParamNames);
    vector<double> motorMaxThrusts(2, motorMaxThrust);
    vector<double> motorMaxRPMs(2, motorMaxRPM);
    vector<double> motorPropPitches(2, motorPropPitch);
    glm::dvec3 motorRightDir = partDirections[4];
    glm::dvec3 motorLeftDir = partDirections[5];
    vector<glm::dvec3> motorThrustDirs = {motorRightDir, motorLeftDir};
    vector<glm::dvec3> motorPos = {POIs[1][1], POIs[2][1]};

    int nPositions = totalForces.size();

    double minPitch = -M_PI/18.0;
    double maxPitch = M_PI/6.0;
    int nPitch = 4;

    double minElevator = -M_PI/6.0;
    double maxElevator = M_PI/6.0;
    int nElevator = 4;

    double minThrottle = 0.2;
    double maxThrottle = 1.0;
    int nThrottle = 5;

    int numConfigs = nPositions * nThrottle;

    //Use formula here to find mean aerodynamic chord: 
    //https://courses.cit.cornell.edu/mae5070/Caughey_2011_04.pdf pg 20
    double wingMeanAeroChord = wingRootChord*(2.0/3.0)*((1.0 + wingScale + wingScale*wingScale)/(1.0+wingScale));
    double totWingArea = wingLength*wingRootChord*(1.0 + wingScale);
    double wingAspectRatio = (4.0*wingLength*wingLength)/totWingArea;


    //Find velocity, and therefore forces, for each configuration
    vector<glm::dvec3> netForces(numConfigs);
    vector<glm::dvec3> netTorques(numConfigs);
    //Magnitudes of forces
    vector<double> netLiftForces(numConfigs);
    vector<double> netPitchTorques(numConfigs);

    
    double weight = G_CONSTANT*mass;

    //Loops over positions (pitch and elevator angles)
    for(int i = 0; i < nPositions; i++){
        //Gets pitch
        double pitch = atan(-positionVariables[i][2]/positionVariables[i][0]);
        double testVelocity = glm::length(glm::dvec3(positionVariables[i][0], positionVariables[i][1], positionVariables[i][2]));
        //double elevatorAngle = positionVariables[i][6];

        //Normalises forces for velocity
        glm::dvec3 adjustedForce = totalForces[i]/(testVelocity*testVelocity);
        glm::dvec3 adjustedTorque = totalTorques[i]/(testVelocity*testVelocity);

        //Adjusts forces to global reference frame
        glm::dquat pitchRot = glm::angleAxis(pitch, glm::dvec3(0.0, -1.0, 0.0));
        glm::dvec3 normalisedForce = glm::inverse(pitchRot) * adjustedForce;
        // Don't need to adjust torque as we are only considering torque in the y direction anyway


        //Loops over throttle values
        for(int t = 0; t < nThrottle; t++){
            double throttle = minThrottle + t*(maxThrottle-minThrottle)/(nThrottle-1);

            double dragCoeff = normalisedForce[0];
            double velocity = calculateVelocity(motorMaxThrusts, motorMaxRPMs, motorPropPitches, 
                throttle, dragCoeff, motorThrustDirs, pitch);
            
            //cout << "velocity: " << velocity << endl; 

            //Finds net force and torque based on velocity
            glm::dvec3 netAeroForce = normalisedForce*velocity*velocity;
            glm::dvec3 netAeroTorque = adjustedTorque*velocity*velocity;

            netForces[i*nThrottle + t] = netAeroForce - glm::dvec3(0.0, 0.0, 1.0)*weight;
            netTorques[i*nThrottle + t] = netAeroTorque;

            //Add forces and torques from each motor
            for(int m = 0; m < (int)motorThrustDirs.size(); m++){
                //Find thrust based on linear model of throttle
                double maxThrust = motorMaxThrusts[m];
                //Thrust force when velocity is zero
                double thrust = 1.3*maxThrust*(throttle/100-0.23);
                //Find pitch speed based on linear model
                double maxRPS = motorMaxRPMs[m]/60;
                double propellorPitch = motorPropPitches[m];
                double pitchSpeed = 0.7*maxRPS*propellorPitch*(throttle/100+0.42);
                
                //Get pitch and yaw of motor
                glm::dvec3 adjustedThrustDir = pitchRot * motorThrustDirs[m];

                //Uses linear model to estimate actual thrust based on velocity
                glm::dvec3 actualThrust = adjustedThrustDir*thrust*(1 - velocity/(pitchSpeed/adjustedThrustDir[0]));
                glm::dvec3 actualTorque = glm::cross(motorPos[m]-COMs[i], actualThrust);

                netForces[i*nThrottle + t] += actualThrust;
                netTorques[i*nThrottle + t] += actualTorque;
            }

            netLiftForces[i*nThrottle + t] = netForces[i*nThrottle + t][2];
            netPitchTorques[i*nThrottle + t] = netTorques[i*nThrottle + t][1];
        }
    }


    
    //Find points at which net lift is zero, and points at which net torque is zero 
    //by computing an SDF with the values equal to the using marching cubes
    MC::mcMesh forceMesh;
    MC::marching_cube(netLiftForces, nThrottle, nPitch, nElevator, forceMesh);

    MC::mcMesh torqueMesh;
    MC::marching_cube(netPitchTorques, nThrottle, nPitch, nElevator, torqueMesh);

    writeMeshToObj("forceMesh.obj", forceMesh);
    writeMeshToObj("torqueMesh.onj", torqueMesh);

    vector<glm::dvec3> intersectionPoints;

    
    //Stores whether edges have already been checked or not
    int numForceVerts = forceMesh.vertices.size();
    int numForceFaces = forceMesh.indices.size()/3;
    int numTorqueVerts = torqueMesh.vertices.size();
    int numTorqueFaces = torqueMesh.indices.size()/3;
    vector<bool> checkedForceEdges(numForceVerts*numForceVerts, false);
    vector<bool> checkedTorqueEdges(numForceVerts*numForceVerts, false);
    for(int i = 0; i < numForceFaces*3; i += 3){
        
        int vert1Ind = forceMesh.indices[i];
        int vert2Ind = forceMesh.indices[i+1];
        int vert3Ind = forceMesh.indices[i+2];
        glm::dvec3 vert1(forceMesh.vertices[vert1Ind][0],
                        forceMesh.vertices[vert1Ind][1],
                        forceMesh.vertices[vert1Ind][2]);
        glm::dvec3 vert2(forceMesh.vertices[vert2Ind][0],
                        forceMesh.vertices[vert2Ind][1],
                        forceMesh.vertices[vert2Ind][2]);
        glm::dvec3 vert3(forceMesh.vertices[vert3Ind][0],
                        forceMesh.vertices[vert3Ind][1],
                        forceMesh.vertices[vert3Ind][2]);

        //Checks that edge has not already been tested 
        if(!checkedForceEdges[vert1Ind*numForceVerts + vert2Ind]){
            checkedForceEdges[vert1Ind*numForceVerts + vert2Ind] = true;

            //Checks whether the edge in force mesh intersects with the face in torque mesh
            vector<glm::dvec3> newIntersectionPoints = findIntersections(vert1, vert2, torqueMesh);
            intersectionPoints.insert(intersectionPoints.end(), newIntersectionPoints.begin(), newIntersectionPoints.end());
        }
        if(!checkedForceEdges[vert2Ind*numForceVerts + vert3Ind]){
            checkedForceEdges[vert2Ind*numForceVerts + vert3Ind] = true;
            vector<glm::dvec3> newIntersectionPoints = findIntersections(vert2, vert3, torqueMesh);
            intersectionPoints.insert(intersectionPoints.end(), newIntersectionPoints.begin(), newIntersectionPoints.end());
        }
        if(!checkedForceEdges[vert3Ind*numForceVerts + vert1Ind]){
            checkedForceEdges[vert3Ind*numForceVerts + vert1Ind] = true;
            vector<glm::dvec3> newIntersectionPoints = findIntersections(vert3, vert1, torqueMesh);
            intersectionPoints.insert(intersectionPoints.end(), newIntersectionPoints.begin(), newIntersectionPoints.end());
        }

    }

    for(int i = 0; i < numTorqueFaces*3; i += 3){
        
        int vert1Ind = torqueMesh.indices[i];
        int vert2Ind = torqueMesh.indices[i+1];
        int vert3Ind = torqueMesh.indices[i+2];
        glm::dvec3 vert1(torqueMesh.vertices[vert1Ind][0],
                        torqueMesh.vertices[vert1Ind][1],
                        torqueMesh.vertices[vert1Ind][2]);
        glm::dvec3 vert2(torqueMesh.vertices[vert2Ind][0],
                        torqueMesh.vertices[vert2Ind][1],
                        torqueMesh.vertices[vert2Ind][2]);
        glm::dvec3 vert3(torqueMesh.vertices[vert3Ind][0],
                        torqueMesh.vertices[vert3Ind][1],
                        torqueMesh.vertices[vert3Ind][2]);

        //Checks that edge has not already been tested 
        if(!checkedTorqueEdges[vert1Ind*numTorqueVerts + vert2Ind]){
            checkedTorqueEdges[vert1Ind*numTorqueVerts + vert2Ind] = true;

            //Checks whether the edge in force mesh intersects with the face in torque mesh
            vector<glm::dvec3> newIntersectionPoints = findIntersections(vert1, vert2, forceMesh);
            intersectionPoints.insert(intersectionPoints.end(), newIntersectionPoints.begin(), newIntersectionPoints.end());
        }
        if(!checkedTorqueEdges[vert2Ind*numTorqueVerts + vert3Ind]){
            checkedTorqueEdges[vert2Ind*numTorqueVerts + vert3Ind] = true;
            vector<glm::dvec3> newIntersectionPoints = findIntersections(vert2, vert3, forceMesh);
            intersectionPoints.insert(intersectionPoints.end(), newIntersectionPoints.begin(), newIntersectionPoints.end());
        }
        if(!checkedTorqueEdges[vert3Ind*numTorqueVerts + vert1Ind]){
            checkedTorqueEdges[vert3Ind*numTorqueVerts + vert1Ind] = true;
            vector<glm::dvec3> newIntersectionPoints = findIntersections(vert3, vert1, forceMesh);
            intersectionPoints.insert(intersectionPoints.end(), newIntersectionPoints.begin(), newIntersectionPoints.end());
        }

    }

    cout << "Intersections: " << intersectionPoints.size() << endl;

    double bestScore = 0.0;
    //First element is pitch, then elevator deflection, then throttle
    array<double, 3> bestConfiguration = {0.0, 0.0, 0.0};

    if(intersectionPoints.size() > 0){
        for(int i = 0; i < (int)intersectionPoints.size(); i++){

            // Estimates MOI using weighted average
            int elevatorLowerIndex = floor(intersectionPoints[i][2]);
            int elevatorUpperIndex = elevatorLowerIndex+1;
            double elevatorLowerDist = intersectionPoints[i][2] - elevatorLowerIndex;
            double elevatorUpperDist = 1 - elevatorLowerDist;
            glm::dmat3 pointMOI = MOIs[elevatorLowerIndex*nPitch] * elevatorUpperDist + 
                MOIs[elevatorUpperIndex*nPitch] * elevatorLowerDist;

            // Estimates COM using weighted average
            glm::dvec3 pointCOM = COMs[elevatorLowerIndex*nPitch] * elevatorUpperDist + 
                COMs[elevatorUpperIndex*nPitch] * elevatorLowerDist;

            double throttle = minThrottle + (maxThrottle-minThrottle)/(nThrottle-1.0)*intersectionPoints[i][0];
            double pitch = minPitch + (maxPitch-minPitch)/(nPitch-1.0)*intersectionPoints[i][1];
            double elevator = minElevator + (maxElevator-minElevator)/(nElevator-1.0)*intersectionPoints[i][2];

            //cout << "throttle: " << throttle << ", alpha: " << alpha << ", elevator: " << elevator << endl;


            //Get values of force when pitch is changed
            int pitchLowerIndex = floor(intersectionPoints[i][1]);
            int pitchUpperIndex = pitchLowerIndex+1;
            int pitchLowerDist = intersectionPoints[i][1] - pitchLowerIndex;
            //int pitchUpperDist = 1 - pitchLowerDist;
            double pitchLower = atan(-positionVariables[pitchLowerIndex][2]/positionVariables[pitchLowerIndex][0]);
            double pitchUpper = atan(-positionVariables[pitchUpperIndex][2]/positionVariables[pitchUpperIndex][0]);
            //Averages forces for upper and lower values of elevator, for upper and lower values of pitch 
            glm::dvec3 forceLowerPitchLowerElevator = totalForces[elevatorLowerIndex*nPitch+pitchLowerIndex];
            glm::dvec3 forceLowerPitchUpperElevator = totalForces[elevatorUpperIndex*nPitch+pitchLowerIndex];
            glm::dvec3 forceLowerPitch = forceLowerPitchLowerElevator*elevatorUpperDist+forceLowerPitchUpperElevator*elevatorLowerDist;
            glm::dvec3 forceUpperPitchLowerElevator = totalForces[elevatorLowerIndex*nPitch+pitchUpperIndex];
            glm::dvec3 forceUpperPitchUpperElevator = totalForces[elevatorUpperIndex*nPitch+pitchUpperIndex];
            glm::dvec3 forceUpperPitch = forceUpperPitchLowerElevator*elevatorUpperDist+forceUpperPitchUpperElevator*elevatorLowerDist;
            //Repeats for torque
            glm::dvec3 torqueLowerPitchLowerElevator = totalTorques[elevatorLowerIndex*nPitch+pitchLowerIndex];
            glm::dvec3 torqueLowerPitchUpperElevator = totalTorques[elevatorUpperIndex*nPitch+pitchLowerIndex];
            glm::dvec3 torqueLowerPitch = torqueLowerPitchLowerElevator*elevatorUpperDist+torqueLowerPitchUpperElevator*elevatorLowerDist;
            glm::dvec3 torqueUpperPitchLowerElevator = totalTorques[elevatorLowerIndex*nPitch+pitchUpperIndex];
            glm::dvec3 torqueUpperPitchUpperElevator = totalTorques[elevatorUpperIndex*nPitch+pitchUpperIndex];
            glm::dvec3 torqueUpperPitch = torqueUpperPitchLowerElevator*elevatorUpperDist+torqueUpperPitchUpperElevator*elevatorLowerDist;

            //Repeats for tail
            glm::dvec3 tailForceLowerPitchLowerElevator = regionForces[elevatorLowerIndex*nPitch+pitchLowerIndex][0] 
                + regionForces[elevatorLowerIndex*nPitch+pitchLowerIndex][1];
            glm::dvec3 tailForceLowerPitchUpperElevator = regionForces[elevatorUpperIndex*nPitch+pitchLowerIndex][0]
                + regionForces[elevatorUpperIndex*nPitch+pitchLowerIndex][1];
            glm::dvec3 tailForceLowerPitch = tailForceLowerPitchLowerElevator*elevatorUpperDist+tailForceLowerPitchUpperElevator*elevatorLowerDist;
            glm::dvec3 tailForceUpperPitchLowerElevator = regionForces[elevatorLowerIndex*nPitch+pitchUpperIndex][0]
                + regionForces[elevatorLowerIndex*nPitch+pitchUpperIndex][1];
            glm::dvec3 tailForceUpperPitchUpperElevator = regionForces[elevatorUpperIndex*nPitch+pitchUpperIndex][0]
                + regionForces[elevatorUpperIndex*nPitch+pitchUpperIndex][0];
            glm::dvec3 tailForceUpperPitch = tailForceUpperPitchLowerElevator*elevatorUpperDist+tailForceUpperPitchUpperElevator*elevatorLowerDist;
            //Repeats for torque
            glm::dvec3 tailTorqueLowerPitchLowerElevator = regionTorques[elevatorLowerIndex*nPitch+pitchLowerIndex][0]
                + regionTorques[elevatorLowerIndex*nPitch+pitchLowerIndex][1];
            glm::dvec3 tailTorqueLowerPitchUpperElevator = regionTorques[elevatorUpperIndex*nPitch+pitchLowerIndex][0]
                + regionTorques[elevatorUpperIndex*nPitch+pitchLowerIndex][1];
            glm::dvec3 tailTorqueLowerPitch = tailTorqueLowerPitchLowerElevator*elevatorUpperDist+tailTorqueLowerPitchUpperElevator*elevatorLowerDist;
            glm::dvec3 tailTorqueUpperPitchLowerElevator = regionTorques[elevatorLowerIndex*nPitch+pitchUpperIndex][0]
                + regionTorques[elevatorLowerIndex*nPitch+pitchUpperIndex][1];
            glm::dvec3 tailTorqueUpperPitchUpperElevator = regionTorques[elevatorUpperIndex*nPitch+pitchUpperIndex][0]
                + regionTorques[elevatorUpperIndex*nPitch+pitchUpperIndex][1];
            glm::dvec3 tailTorqueUpperPitch = tailTorqueUpperPitchLowerElevator*elevatorUpperDist+tailTorqueUpperPitchUpperElevator*elevatorLowerDist;

            //Find tail efficiency factors for different elevator angles and does weighted average
            //Test velocity is assumed to be the same for all simulations
            double testVelocity = glm::length(glm::dvec3(positionVariables[elevatorLowerIndex*nPitch][0],
                positionVariables[elevatorLowerIndex*nPitch][1],positionVariables[elevatorLowerIndex*nPitch][2]));
            double tailUpstreamVelocityLower = regionVelMags[elevatorLowerIndex*nPitch][0];
            double tailUpstreamVelocityUpper = regionVelMags[elevatorUpperIndex*nPitch][0];
            double tailUpstreamVelocity = tailUpstreamVelocityLower*elevatorUpperDist + 
                tailUpstreamVelocityUpper*elevatorLowerDist;

            double tEfficiencyUpper = (tailUpstreamVelocity/testVelocity)*(tailUpstreamVelocity/testVelocity);
            double tEfficiencyLower = (tailUpstreamVelocity/testVelocity)*(tailUpstreamVelocity/testVelocity);
            double tEfficiencyFactor = tEfficiencyLower * elevatorUpperDist + tEfficiencyUpper * elevatorLowerDist;
            

            //Finds drag coefficient using weighted average
            //Finds test velocity 
            glm::dvec3 forceDiagLower = totalForces[elevatorLowerIndex*nPitch + pitchLowerIndex];
            glm::dvec3 forceDiagUpper = totalForces[elevatorUpperIndex*nPitch + pitchUpperIndex];
            
            //Gets the diagonal distances of the values to the nearest index
            double diagPitchElevLowerDist = sqrt(elevatorLowerDist*elevatorLowerDist+pitchLowerDist*pitchLowerDist)/sqrt(2.0);
            double diagPitchElevUpperDist = 1 - diagPitchElevLowerDist;
            glm::dvec3 forceDiag = forceDiagLower * diagPitchElevUpperDist + forceDiagUpper * diagPitchElevLowerDist;
            double dragCoeff = forceDiag[0]/(testVelocity*testVelocity);
            
            
            double velocity = calculateVelocity(motorMaxThrusts, motorMaxRPMs, motorPropPitches,
                throttle, dragCoeff, motorThrustDirs, pitch);


            //Estimation of Mq:
            //https://courses.cit.cornell.edu/mae5070/Caughey_2011_04.pdf pg 37
            //Equation for V_H and tail efficiency factor are on pg 32
            //Non-dimensionalisation on pg 11-12
            //Pitch rate definitions on pg 36
            //St is tail planform area (flattened area)
            //S is total wing planform area (both wings combined) (approximate as trapezoid pg 19)
            //Approximation of c bar (wing mean Waerodynamic chord) on pg 18
            //Span is sum of both wing lengths
            //Redimensionalisation of CMq on pg 60

            //Estimation of aerodynamic centres and calculation of tail lever arm:
            //https://www.sciencedirect.com/topics/engineering/aerodynamic-center
            //A more generalised formula can be used about the COM instead of the quarter chord

            double Qref = 0.5*RHO*velocity*velocity;
            double tailLeverArm;
            {
                //Approximate alpha using pitch as aircraft is assumed to be in trim flight
                double dAlpha = pitchUpper - pitchLower;

                //Derivative of pitching moment with respect to alpha
                //Includes non-dimensionalisation
                double dCMdalphaWing = (torqueUpperPitch[1] - torqueLowerPitch[1])/(dAlpha * Qref *  totWingArea * wingMeanAeroChord);
                double dCMdalphaTail = (tailTorqueUpperPitch[1] - tailTorqueLowerPitch[1])/(dAlpha * Qref * totWingArea * wingMeanAeroChord);

                //Derivative of lift with respect to alpha
                double dCLdalphaWing = (forceUpperPitch[2] - forceLowerPitch[2])/(dAlpha * Qref * totWingArea);
                double dCLdalphaTail = (tailForceUpperPitch[2] - tailForceLowerPitch[2])/(dAlpha * Qref * totWingArea);

                //Adds distance from reference point as fraction of wing aero chord, multiplied by wing aero chord,
                //to the reference point
                double aeroCentreWingX = pointCOM[0] - wingMeanAeroChord*dCMdalphaWing/dCLdalphaWing;
                double aeroCentreTailX = pointCOM[0] - wingMeanAeroChord*dCMdalphaTail/dCLdalphaTail;

                tailLeverArm = aeroCentreWingX - aeroCentreTailX;
            }

            //cout << "lever arm: " << tailLeverArm << endl;

            //Approximate alpha using pitch as aircraft is assumed to be in trim flight
            double dAlpha = pitchUpper - pitchLower;

            //Ensures that point is a stable rather than unstable equilibrium (M_alpha < 0)
            double dMdAlpha = (torqueUpperPitch[1]-torqueLowerPitch[1])/dAlpha;

            //Nondimensionalises forces
            double tailLiftCoeffLower = tailForceLowerPitch[2]/(Qref*tailHorizArea);
            double tailLiftCoeffUpper = tailForceUpperPitch[2]/(Qref*tailHorizArea);

            //Gets change in tail angle of attack
            //Tail angle of attack formula
            //https://ocw.tudelft.nl/wp-content/uploads/Hand-out-Stability_01.pdf pg4
            //Downwash angle formula
            //https://www.sciencedirect.com/topics/engineering/downwash-angle
            double liftCoeffLower = forceLowerPitch[2]/(Qref*totWingArea);
            double liftCoeffUpper = forceUpperPitch[2]/(Qref*totWingArea);
            double epsilonLower = 2.0*liftCoeffLower/(M_PI*wingAspectRatio);
            double epsilonUpper = 2.0*liftCoeffUpper/(M_PI*wingAspectRatio);
            double alphaTLower = pitchLower - epsilonLower;
            double alphaTUpper = pitchUpper - epsilonUpper;
            double dAlphaT = alphaTUpper - alphaTLower;

            //Derivative of tail lift coefficient with respect to tail angle of attack
            double dC_L_tdalphaT = (tailLiftCoeffUpper - tailLiftCoeffLower)/dAlphaT;
            
            double tailVolumeParameter = tailLeverArm*tailHorizArea/(wingMeanAeroChord*totWingArea);
            double Iyy = pointMOI[1][1];

            //Approximates pitch damping
            double CMq = -2.0*tEfficiencyFactor*(tailLeverArm/wingMeanAeroChord)*
                tailVolumeParameter*dC_L_tdalphaT;
            //Dimensionalises coefficient
            double Mq = Qref*totWingArea*wingMeanAeroChord*wingMeanAeroChord*CMq/(2*Iyy*velocity);

            //Approximates M_\dot{alpha} by assuming it is much smaller than Mq 
            //(refer to example values in above youtube video)
            double MalphaDot = 0.15*Mq;

            //Gets oscillation frequency and damping coefficient
            //Equations:
            //https://www.youtube.com/watch?v=-P7XWCBgKAo

            
            double score = 0.0;
            array<double, 3> currentConfig = {pitch, elevator, throttle};

            if(dMdAlpha < 0.0){
                //Oscillation frequency and damping coefficient
                double omega_n = sqrt(-dMdAlpha/Iyy);
                double zeta = -(Mq + MalphaDot)/(2*Iyy*omega_n );
                
                //Calculates score based on calcualted coefficients
                score = zeta*(1.0/omega_n)*(-1.0/forceDiag[0]);
            }

            if(score > bestScore){
                bestScore = score;
                bestConfiguration = currentConfig;
            }

        }

    }else{
        cout << "No configuration found that enables trim flight" << endl;
        bestScore = 0.0;
        bestConfiguration = {0.0, 0.0, 0.0};
    }

    cout << "Best Config, alpha: " << bestConfiguration[0] << ", elevator: " << bestConfiguration[1]
    << ", throttle: " << bestConfiguration[2] << endl;

    return bestScore;
};

//Finds intersection points between a line segment and a mesh
std::vector<glm::dvec3> findIntersections(glm::dvec3 vert1, glm::dvec3 vert2, MC::mcMesh mesh){
    int numFaces = mesh.indices.size()/3;

    std::vector<glm::dvec3> intersectionPoints;

    for(int i = 0; i < numFaces*3; i += 3){
                
        //Gets vertices of triangle
        glm::dvec3 triVert1 = mesh.vertices[mesh.indices[i]];
        glm::dvec3 triVert2 = mesh.vertices[mesh.indices[i + 1]];
        glm::dvec3 triVert3 = mesh.vertices[mesh.indices[i + 2]];

        //Check for intersection using: https://stackoverflow.com/questions/42740765/intersection-between-line-and-triangle-in-3d
        //SignedVolume(a,b,c,d) = (1.0/6.0)*dot(cross(b-a,c-a),d-a);

        double signedVol1 = glm::dot(glm::cross(triVert1-vert1, triVert2-vert1), triVert3-vert1)/6.0;
        double signedVol2 = glm::dot(glm::cross(triVert1-vert2, triVert2-vert2), triVert3-vert2)/6.0;

        double signedVol3 = glm::dot(glm::cross(vert2-vert1, triVert1-vert1), triVert2-vert1)/6.0;
        double signedVol4 = glm::dot(glm::cross(vert2-vert1, triVert2-vert1), triVert3-vert1)/6.0;
        double signedVol5 = glm::dot(glm::cross(vert2-vert1, triVert3-vert1), triVert1-vert1)/6.0;

        //Checks if points are on opposite sides of the triangle, and that the long passes through the triangle
        if((std::signbit(signedVol1) != std::signbit(signedVol2)) && ((signedVol3 == signedVol4) && (signedVol4 == signedVol5))){
            //Finds specific point of intersection

            glm::dvec3 N = glm::cross(triVert2-triVert1, triVert3-triVert1);
            //Parametric value of intersection along line
            double t = -glm::dot(vert1-triVert1,N)/glm::dot(vert2-vert1,N);
            glm::dvec3 intersectionPoint = vert1 + t*(vert2-vert1);
            intersectionPoints.push_back(intersectionPoint);
        }

    }

    return intersectionPoints;
}

// Calculates stable velocity point of plane based on drag and motor thrust
// (motor thrust points in the forwards direction)
double calculateVelocity(vector<double> motorMaxThrusts, vector<double> motorMaxRPMs, 
    vector<double> motorPropPitches, double throttle, double dragCoeff, 
    vector<glm::dvec3> motorThrustDirs, double pitch){

    //av^2 + bv + c = 0
    //C_D v^2 + T * (1 - v/p) = 0
    // dragCoeff is the normalised force in x direction, and so is probably negative
    double a = -dragCoeff;
    double b = 0.0;
    double c = 0.0;

    //Sum force from each motor to get coefficients of quadratic equation involving velocity
    int nMotors = motorMaxThrusts.size();
    for(int m = 0; m < nMotors; m++){

        double maxThrust = motorMaxThrusts[m];
        double maxRPS = motorMaxRPMs[m]/60.0;
        double propellorPitch = motorPropPitches[m]*0.025;

        //Find thrust and pitch speed based on linear model
        double thrust = 1.3*maxThrust*(throttle/100.0-0.23);
        double pitchSpeed = 0.7*maxRPS*propellorPitch*(throttle/100.0+0.42);
        
        //Get pitch and yaw of motor
        glm::dquat pitchRot =  glm::angleAxis(pitch, glm::dvec3(0.0, -1.0, 0.0));
        glm::dvec3 adjustedThrustDir = pitchRot * motorThrustDirs[m];

        //Pitch speed is divided by thrust direction x component to account 
        //for reduced parallel air velocity when at an angle
        pitchSpeed /= adjustedThrustDir[0];

        //Fraction of thrust in horizontal direction
        double horizThrust = thrust*adjustedThrustDir[0];
        
        
        b += horizThrust/pitchSpeed;
        c += -horizThrust;
    }

    //Solve quadratic equation. The two solutions are roughly centred around zero, with
    //one positive and one negative. Solve for the positive one
    double velocity = (-b+sqrt(b*b-4*a*c))/(2*a);

    return velocity;
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
    vector<double> pointMass = {batteryMass};
    vector<glm::dvec3> massLocation = {glm::dvec3(0.0, 0.0, batteryPos*fuselageLength)};

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
    double motorMass = getParam("motorMass", paramVals, paramNames);


    vector<double> zSampleVals = {0.0, -motorPodLength};
    vector<glm::dvec2> scaleVals = {glm::dvec2(1.0, 1.0), glm::dvec2(1.0, 1.0)};
    vector<glm::dvec2> posVals = {glm::dvec2(0.0, 0.0), glm::dvec2(0.0, 0.0)};


    glm::dvec3 pivotPoint = glm::dvec3(0.0, 0.0, 0.0);
    glm::dquat rotation = glm::dquat(glm::dvec3(0.0, 0.5*M_PI, 0.0));
    glm::dvec3 translation = glm::dvec3(motorPodXPos, 0.0, -motorPodYPos);

    glm::dvec3 motorDir = glm::dvec3(0.0, 0.0, 1.0);
    vector<double> pointMass = {motorMass};
    vector<glm::dvec3> massLocation = {glm::dvec3(0.0, 0.0, -motorPodLength + 0.5*motorLength)};

    extrusionData extrusion(zSampleVals, posVals, scaleVals, rotation, translation, 
        pivotPoint, pointMass, massLocation, motorDir);

    return extrusion;
}

extrusionData extrudeLeftMotorPod(vector<string> paramNames, vector<double> paramVals, double meshRes){

    double motorPodLength = getParam("motorPodLength", paramVals, paramNames);
    double motorPodXPos = getParam("motorPodXPos", paramVals, paramNames);
    double motorPodYPos = getParam("motorPodYPos", paramVals, paramNames);
    double motorLength = getParam("motorLength", paramVals, paramNames);
    double motorMass = getParam("motorMass", paramVals, paramNames);


    vector<double> zSampleVals = {0.0, -motorPodLength};
    vector<glm::dvec2> scaleVals = {glm::dvec2(1.0, 1.0), glm::dvec2(1.0, 1.0)};
    vector<glm::dvec2> posVals = {glm::dvec2(0.0, 0.0), glm::dvec2(0.0, 0.0)};


    glm::dvec3 pivotPoint = glm::dvec3(0.0, 0.0, 0.0);
    glm::dquat rotation = glm::dquat(glm::dvec3(0.0, 0.5*M_PI, 0.0));
    glm::dvec3 translation = glm::dvec3(motorPodXPos, 0.0, motorPodYPos);

    glm::dvec3 motorDir = glm::dvec3(0.0, 0.0, 1.0);
    vector<double> pointMass = {motorMass};
    vector<glm::dvec3> massLocation = {glm::dvec3(0.0, 0.0, -motorPodLength + 0.5*motorLength)};

    extrusionData extrusion(zSampleVals, posVals, scaleVals, rotation, translation, 
        pivotPoint, pointMass, massLocation, motorDir);

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

    extrusionData extrusion(zSampleVals, posVals, scaleVals, rotation, translation, 
        pivotPoint, true);

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