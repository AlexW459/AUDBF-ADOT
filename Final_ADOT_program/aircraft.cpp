#include "aircraft.h"

#define MC_CPP_USE_DOUBLE_PRECISION
#define MC_IMPLEM_ENABLE
#include "Mesh_Generation/MC.h"

aircraft::aircraft(vector<string> _paramNames, vector<dataTable> _discreteTables,
    function<void(vector<string>&, vector<double>&, const vector<dataTable>&, 
    vector<int>, double&, double&, double&, double&)> _derivedParamsFunc,
    vector<function<profile(vector<string>, vector<double>, double)>> _profileFunctions, double _roughnessHeight){
    
    //Set class variables
    parameterNames = _paramNames;
    derivedParamsFunc = _derivedParamsFunc;
    profileFunctions = _profileFunctions;
    discreteTables = _discreteTables;
    roughnessHeight = _roughnessHeight;
}

void aircraft::addPart(string partName, double density, 
    function<extrusionData(vector<string>, vector<double>, double)>extrusionFunction, int profileIndex){
    //Adds first part to tree
    partNames.push_back(partName);
    parentIndices.push_back({});
    //First part cannot be a control surface
    extrusionFunctions.push_back(extrusionFunction);
    partDensities.push_back(density);
    partProfiles.push_back(profileIndex);
}

void aircraft::addPart(string partName, string parentName, double density,
    function<extrusionData(vector<string>, vector<double>, double)> extrusionFunction, int profileIndex){

    //Gets index of parent
    int parentIndex = findPart(parentName);
    //Adds name of new part to list
    partNames.push_back(partName);
    
    //Adds parent index to list
    vector<int> partParentIndices = {parentIndex};
    partParentIndices.insert(partParentIndices.begin() + 1, parentIndices[parentIndex].begin(), 
        parentIndices[parentIndex].end());
    parentIndices.push_back(partParentIndices);

    extrusionFunctions.push_back(extrusionFunction);
    partDensities.push_back(density);
    partProfiles.push_back(profileIndex);

}


int aircraft::findPart(string partName){
    //Gets index of part
    int partIndex = (int)(find(partNames.begin(), partNames.end(), partName) - partNames.begin());
    //Checks that specified part exists
    if(partIndex == (int)partNames.size())
        throw std::runtime_error("Could not find part \"" + partName + "\" in function aircraft::findPart");
    else
        return partIndex;
}


double aircraft::calculateScore(vector<double> paramVals, vector<int> discreteVals, 
    function<double(array<double, 3>, double, glm::dvec3, double, double, double,
    double, vector<string>, vector<double>)> scoreFunc, array<double, 3>& bestConfig,
    double volMeshRes, double surfMeshRes, int procRank, int nSimNodes, int nSimTasksPerNode){

    if(procRank == 0) {
        string deleteDirBash = "rm -r -f Aerodynamics_Simulation_*" + to_string(procRank);
        int failure = system(deleteDirBash.c_str());
    	if(failure) throw runtime_error("Failed to clean old case directories on " + to_string(procRank) + " before simulations");

	cout << "cleaned old directories" << endl;
    }

    MPI_Barrier(MPI_COMM_WORLD);

    //Creates a unique case directory
    string copyBash = "cp -r Aerodynamics_Simulation Aerodynamics_Simulation_" + to_string(procRank);
    int failure = system(copyBash.c_str());
    if(failure) throw runtime_error("Failed to copy simulation directory for case " + to_string(procRank));

    //cout << "made a copy!" << endl;

    //Gets derived parameter values. Values and names are inserted onto the end of argument vectors
    vector<string> paramNames = parameterNames;
    double wingRootChord;
    double wingLength;
    double wingScale;
    double tailHorizArea;
    derivedParamsFunc(paramNames, paramVals, discreteTables, discreteVals, wingRootChord, 
        wingLength, wingScale, tailHorizArea);

    //Use formula here to find mean aerodynamic chord: 
    //https://courses.cit.cornell.edu/mae5070/Caughey_2011_04.pdf pg 20
    double wingMeanAeroChord = wingRootChord*(2.0/3.0)*((1.0 + wingScale + wingScale*wingScale)/(1.0+wingScale));
    double totWingArea = wingLength*wingRootChord*(1.0 + wingScale);
    double wingAspectRatio = (4.0*wingLength*wingLength)/totWingArea;



    //Gets extrusion information and finds relevant values
    int numParts = partNames.size();
    vector<profile> profiles;
    vector<extrusionData> extrusions;
    //cout << "Getting extrusion data" << endl;
    getExtrusionData(profiles, extrusions, paramNames, paramVals, discreteVals, volMeshRes); 


    glm::dvec3 COMSoFar(0.0);
    double massSoFar = 0.0;
    double staticMassSoFar = 0.0;
    glm::dmat3 MOISoFar(0.0);

    vector<glm::dmat2x3> boundingBoxes(numParts);
    glm::dmat2x3 totalBoundingBox(50.0, 50.0, 50.0, -50.0, -50.0, -50.0);

    //Stores values relating to control surfaces
    vector<glm::dmat3> controlMOIs;
    vector<glm::dvec3> controlCOMs;
    vector<double> controlMasses;
    vector<glm::dvec3> controlPivotPoints;
    vector<glm::dvec3> controlAxes;
    vector<int> controlSurfaces;
    vector<int> staticSurfaces;

    //Stores values relating to propellors
    vector<int>        motorParts;
    vector<glm::dvec3> motorThrustDirs;
    vector<glm::dvec3> propellersPos;

    //Stores values relating to horizontal stabilisers
    int horizontalStabiliserPart = 0;
    int elevatorPart = 0;

    cout << "Getting vol vals on rank " << procRank << endl;
    for(int i = 0; i < numParts; i++){

        //Calculate variables based on profile and extrusion data
        glm::dvec3 partCOM;
        glm::dmat3 partMOI;
        double partMass;
        int profileIndex = partProfiles[i];
        glm::dmat2x3 boundingBox(10.0, 10.0, 10.0, -10.0, -10.0, -10.0);
        getVolVals(profiles[profileIndex], extrusions[i], partDensities[i], partMass, partCOM, partMOI, boundingBox);

        //cout << "part bounding box: " << boundingBox[0][0] << ", " << boundingBox[0][1] << ", " << boundingBox[0][2] << " - "
        //    << boundingBox[1][0] << ", " << boundingBox[1][1] << ", " << boundingBox[1][2] << endl;

        //Get transformations applied to part
        vector<int> partParentIndices = parentIndices[i];
        //Adds part itself to list of transformations
        vector<int> transformIndices = {i};
        transformIndices.insert(transformIndices.begin() + 1, partParentIndices.begin(), partParentIndices.end());

        //If it is a control surface
        glm::dvec3 partPivot = extrusions[i].pivotPoint;
        glm::dvec3 partAxis = extrusions[i].controlAxis;

        //If it is a motor
        glm::dvec3 partThrustDir = extrusions[i].motor.thrustDir;
        glm::dvec3 partPropPos = extrusions[i].motor.propPos;

        //Applies transformations to part
        for(int p = 0; p < (int)transformIndices.size(); p++){
            int tIndex = transformIndices[p];

            partCOM -= extrusions[tIndex].pivotPoint;
            partCOM = extrusions[tIndex].rotation * partCOM;
            partCOM += extrusions[tIndex].translation + extrusions[tIndex].pivotPoint;

            //Performs reverse operations on the MOI because these equations actually move the point
            //around which the part rotates, 
            glm::dmat3 returnPivotMat = constructRelationMatrix(-1.0*extrusions[tIndex].pivotPoint);
            partMOI += partMass*returnPivotMat;
            glm::dmat3 rotMat = glm::mat3_cast(extrusions[tIndex].rotation);
            partMOI = rotMat*partMOI*(glm::transpose(rotMat));
            glm::dmat3 transMat = constructRelationMatrix(extrusions[tIndex].translation + 
                extrusions[tIndex].pivotPoint);
            partMOI += partMass*transMat;

            //Apply transformations to pivot point and axis of rotation
            partPivot -= extrusions[tIndex].pivotPoint;
            partPivot = extrusions[tIndex].rotation * partPivot;
            partPivot += extrusions[tIndex].translation + extrusions[tIndex].pivotPoint;

            partAxis = extrusions[tIndex].rotation * partAxis;

            partPropPos -= extrusions[tIndex].pivotPoint;
            partPropPos = extrusions[tIndex].rotation * partPropPos;
            partPropPos += extrusions[tIndex].translation + extrusions[tIndex].pivotPoint;
            partThrustDir = extrusions[tIndex].rotation * partThrustDir;


            //Apply transformations to bounding box
            boundingBox[0] = boundingBox[0] - extrusions[tIndex].pivotPoint;
            boundingBox[1] = boundingBox[1] - extrusions[tIndex].pivotPoint;
            boundingBox[0] = extrusions[tIndex].rotation * boundingBox[0];
            boundingBox[1] = extrusions[tIndex].rotation * boundingBox[1];
            boundingBox[0] = boundingBox[0] + extrusions[tIndex].translation + extrusions[tIndex].pivotPoint;
            boundingBox[1] = boundingBox[1] + extrusions[tIndex].translation + extrusions[tIndex].pivotPoint;

        }

        //Rearranges coordinates of bounding box to account for the fact that the maximum and minimum coordinates
        //may have switched positions during the rotations
        glm::dvec3 newMinBound = min(boundingBox[0], boundingBox[1]);
        glm::dvec3 newMaxBound = max(boundingBox[0], boundingBox[1]);
        boundingBox[0] = newMinBound;
        boundingBox[1] = newMaxBound;

        //Adds 5% in every direction to bounding box
        double margin = 0.05;
        glm::dvec3 boundSize = boundingBox[1] - boundingBox[0];
        boundingBox[0] -= boundSize*margin;
        boundingBox[1] += boundSize*margin;


        //Adjusts total bounding box if necessary
        totalBoundingBox[0] = min(totalBoundingBox[0], boundingBox[0]);
        totalBoundingBox[1] = max(totalBoundingBox[1], boundingBox[1]);

        boundingBoxes[i] = boundingBox;

        massSoFar += partMass;

        if(!extrusions[i].isControl){
            //Adds to total COM
            COMSoFar += partCOM*partMass;
            //Adds to total mass
            staticMassSoFar += partMass;
            MOISoFar += partMOI;

            if(!extrusions[i].isHorizontalStabiliser){
                //Makes list of non-control surfaces
                staticSurfaces.push_back(i);
            }

        }else{
            controlSurfaces.push_back(i);
            controlCOMs.push_back(partCOM*partMass);
            controlMOIs.push_back(partMOI);
            controlMasses.push_back(partMass);
            controlAxes.push_back(partAxis);
            controlPivotPoints.push_back(partPivot);

            if(extrusions[i].isHorizontalStabiliser){
                elevatorPart = i;
            }
        }

        if(extrusions[i].isMotor){
            motorParts.push_back(i);
            motorThrustDirs.push_back(partThrustDir);
            propellersPos.push_back(partPropPos);
        }

        if(extrusions[i].isHorizontalStabiliser && !extrusions[i].isControl){
            horizontalStabiliserPart = i;
        }


    }

    //Increases total bounding box just in case
    totalBoundingBox[0] -= glm::dvec3(0.2, 0.2, 0.2);
    totalBoundingBox[1] += glm::dvec3(0.2, 0.2, 0.2);

    double totalMass = massSoFar;

    //Gets COM
    double staticMass = staticMassSoFar;
    glm::dvec3 staticCOM = COMSoFar/staticMass;
    glm::dmat3 staticMOI = MOISoFar;

    //Gets SDF
    //staticSurfaces.insert(staticSurfaces.end(), controlSurfaces.begin(), controlSurfaces.end());

    //cout << "Initialising SDF" << endl;
    //Inits SDF
    vector<double> SDF;
    vector<glm::dvec3> XYZ;
    glm::ivec3 SDFSize = initSDF(SDF, XYZ, totalBoundingBox, surfMeshRes);
    //Adds parts
    vector<double> staticSDF = updateSDF(SDF, SDFSize, XYZ, profiles, partProfiles, extrusions, parentIndices,
        boundingBoxes, totalBoundingBox, staticSurfaces, surfMeshRes);

    vector<double> stabiliserSDF = updateSDF(SDF, SDFSize, XYZ, profiles, partProfiles, extrusions, parentIndices,
        boundingBoxes, totalBoundingBox, {horizontalStabiliserPart}, surfMeshRes);;


    //Copies SDF without control surfaces for future use

    //applyGaussianBlur(0.2, 7, staticSDF, SDFSize);


    //Specify the different values to be tested
    array<double, 2> alphaRange = {-M_PI/18.0, 3*M_PI/18.0};
    int nAlpha= 4;
    array<double, 2> elevatorRange = {-M_PI/6.0, M_PI/6.0};
    int nElevator = 4;
    array<double, 2> throttleRange = {60.0, 100.0};
    int nThrottle = 5;

    //Fill tables with required configurations
    int nPositions = nAlpha*nElevator;
    vector<vector<double>> positionVariables(nPositions);

    //Loop over each possible variation of AOA and elevator deflection
    //Group configs with the same elevator deflection together as remeshing is
    //not required between these configs
    
    for(int e = 0; e < nElevator; e++){
        for(int a = 0; a < nAlpha; a++){

            //Set configuration
            double alpha = alphaRange[0] + a*(alphaRange[1]-alphaRange[0])/(nAlpha-1);
            double elevatorPos = elevatorRange[0] + e*(elevatorRange[1]-elevatorRange[0])/(nElevator-1);
        
            positionVariables[e*nAlpha+a] = {alpha, 0, elevatorPos};

        }
    }


    vector<glm::dvec3> COMs;
    vector<glm::dmat3> MOIs;

    //Use getPhysVals to find COMs and MOIs for each configuration
    getPhysVals(positionVariables, staticMass, staticCOM, staticMOI, controlMasses, 
        controlPivotPoints, controlAxes, controlCOMs, controlMOIs, COMs, MOIs);


    cout << "Getting table of aerovals on rank " << procRank << endl;
    //Use getAeroVals to get force coefficients of each configuration (force divided by velocity squared)
    vector<double> dummyVar;
    vector<glm::dvec3> tailForceList, tailTorqueList;\
    pair<vector<glm::dvec3>, vector<glm::dvec3>> aeroTable = getAeroVals(positionVariables, 
        staticSDF, SDFSize, XYZ, profiles, extrusions, controlSurfaces, controlAxes, 
        controlPivotPoints, horizontalStabiliserPart, elevatorPart, stabiliserSDF,
        COMs, boundingBoxes, totalBoundingBox, dummyVar, tailForceList, tailTorqueList, 
        surfMeshRes, procRank, nSimNodes, nSimTasksPerNode);


    vector<glm::dvec3> netAeroForces(nPositions);
    vector<glm::dvec3> netAeroTorques(nPositions);

    //Combines forces
    for(int i = 0; i < nPositions; i++){
        netAeroForces[i] = aeroTable.first[i] + tailForceList[i];
        netAeroTorques[i] = aeroTable.second[i] + tailTorqueList[i];
    }

    int numConfigs = nPositions * nThrottle;
    //Find velocity, and therefore forces, for each configuration
    vector<glm::dvec3> netForces(numConfigs);
    vector<glm::dvec3> netTorques(numConfigs);
    //Magnitudes of forces
    vector<double> netLiftForces(numConfigs);
    vector<double> netPitchTorques(numConfigs);

    double weight = G_CONSTANT*totalMass;

    for(int i = 0; i < nPositions; i++){
        double alpha = positionVariables[i][0];
        double yaw = positionVariables[i][1];

        //Loops over throttle values
        for(int t = 0; t < nThrottle; t++){
            double throttle = throttleRange[0] + t*(throttleRange[1]-throttleRange[0])/(nThrottle-1);

            double velocity = calculateVelocity(extrusions, motorParts, throttle, 
                netAeroForces[i][0], motorThrustDirs, alpha, yaw);
            
            //cout << "velocity: " << velocity << endl; 

            //Finds net force and torque based on velocity
            glm::dvec3 netAeroForce = netAeroForces[i]*velocity*velocity;
            glm::dvec3 netAeroTorque = netAeroTorques[i]*velocity*velocity;

            netForces[i*nThrottle + t] = netAeroForce - glm::dvec3(0.0, 0.0, 1.0)*weight;
            netTorques[i*nThrottle + t] = netAeroTorque;

            //Add forces and torques from each motor
            for(int m = 0; m < (int)motorParts.size(); m++){
                //Find thrust based on linear model of throttle
                double maxThrust = extrusions[motorParts[m]].motor.maxThrust;
                //Thrust force when velocity is zero
                double thrust = 1.3*maxThrust*(throttle/100-0.23);
                //Find pitch speed based on linear model
                double maxRPS = extrusions[motorParts[m]].motor.maxRPM/60;
                double propellorPitch = extrusions[motorParts[m]].motor.propPitch*0.025;
                double pitchSpeed = 0.7*maxRPS*propellorPitch*(throttle/100+0.42);
                
                //Get pitch and yaw of motor
                double motorPitch = alpha + atan(motorThrustDirs[m][2]/motorThrustDirs[m][0]);
                double motorYaw = yaw + atan(motorThrustDirs[m][1]/motorThrustDirs[m][0]);

                //Uses linear model to estimate actual thrust based on velocity
                glm::dvec3 actualThrust = glm::dvec3(-thrust*cos(motorPitch)*cos(motorYaw), 
                            -thrust*sin(motorYaw)*sin(motorPitch), 
                            thrust*sin(motorPitch))*(1 - velocity/(pitchSpeed/(cos(motorPitch)*cos(motorYaw))));
                glm::dvec3 actualTorque = glm::cross(propellersPos[m]-COMs[i], actualThrust);

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
    MC::marching_cube(netLiftForces, nThrottle, nAlpha, nElevator, forceMesh);

    MC::mcMesh torqueMesh;
    MC::marching_cube(netPitchTorques, nThrottle, nAlpha, nElevator, torqueMesh);

    writeMeshToObj("forceMesh.obj", forceMesh);
    writeMeshToObj("torqueMesh.onj", torqueMesh);


    //Find intersections between two meshes by checking for intersections between every edge of
    //one mesh and the faces of the opposite mesh and vice versa
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

    cout << "Intersections on rank " << procRank << ": " << intersectionPoints.size() << endl;

    double bestScore = 0.0;
    //First element is pitch, then elevator deflection, then throttle
    array<double, 3> bestConfiguration = {0.0, 0.0, 0.0};

    if(intersectionPoints.size() > 0){
        for(int i = 0; i < (int)intersectionPoints.size(); i++){

            //Converts points into config variables
            double throttle = throttleRange[0] + (throttleRange[1]-throttleRange[0])/(nThrottle-1.0)*intersectionPoints[i][0];
            double alpha = alphaRange[0] + (alphaRange[1]-alphaRange[0])/(nAlpha-1.0)*intersectionPoints[i][1];
            double elevator = elevatorRange[0] + (elevatorRange[1]-elevatorRange[0])/(nElevator-1.0)*intersectionPoints[i][2];

            cout << "throttle: " << throttle << ", alpha: " << alpha << ", elevator: " << elevator << endl;

            //Runs simulation around point to get stability derivatives
            vector<vector<double>> pointPosVals = {{alpha-M_PI/18.0, 0.0, elevator},
                                                    {alpha, 0.0, elevator},
                                                    {alpha+M_PI/18.0, 0.0, elevator}};

            vector<glm::dvec3> pointCOMs;
            vector<glm::dmat3> pointMOIs;
            getPhysVals(pointPosVals, staticMass, staticCOM, staticMOI, controlMasses, 
                controlPivotPoints, controlAxes, controlCOMs, controlMOIs, pointCOMs, pointMOIs);


            vector<double> tEfficiencyFactors;
            vector<glm::dvec3> tailForces, tailTorques;
            pair<vector<glm::dvec3>, vector<glm::dvec3>> pointAeroTable = getAeroVals(positionVariables, 
                staticSDF, SDFSize, XYZ, profiles, extrusions, controlSurfaces, controlAxes, 
                controlPivotPoints, horizontalStabiliserPart, elevatorPart, stabiliserSDF,
                COMs, boundingBoxes, totalBoundingBox, tEfficiencyFactors, tailForces, tailTorques, 
                surfMeshRes, procRank, nSimNodes, nSimTasksPerNode);

            vector<glm::dvec3> aeroForces = pointAeroTable.first;
            vector<glm::dvec3> aeroTorques = pointAeroTable.second;
            
            //Change in angle of attack
            double alphaLower = pointPosVals[2][0];
            double alphaUpper = pointPosVals[0][0];
            double dAlpha = alphaUpper - alphaLower;

            //Finds velocity at point
            double dragCoeff = aeroForces[i][0] + tailForces[i][0];
            double velocity = calculateVelocity(extrusions, motorParts, throttle, dragCoeff, motorThrustDirs, alpha, 0.0);

            //Gets forces at point
            //AOA is positive when pitch is negative
            glm::dvec3 forceLower = aeroForces[2]*velocity*velocity;
            glm::dvec3 forceUpper = aeroForces[0]*velocity*velocity;
            glm::dvec3 torqueLower = aeroTorques[2]*velocity*velocity;
            glm::dvec3 torqueUpper = aeroTorques[0]*velocity*velocity;

            glm::dvec3 tailForceLower = tailForces[2]*velocity*velocity;
            glm::dvec3 tailForceUpper = tailForces[0]*velocity*velocity;
            glm::dvec3 tailTorqueLower = tailTorques[2]*velocity*velocity;
            glm::dvec3 tailTorqueUpper = tailTorques[0]*velocity*velocity;



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
                //Derivative of pitching moment with respect to alpha
                //Includes non-dimensionalisation
                double dCMdalphaWing = (torqueUpper[1] - torqueLower[1])/(dAlpha * Qref *  totWingArea * wingMeanAeroChord);
                double dCMdalphaTail = (tailTorqueUpper[1] - tailTorqueLower[1])/(dAlpha * Qref * totWingArea * wingMeanAeroChord);

                //Derivative of lift with respect to alpha
                double dCLdalphaWing = (forceUpper[2] - forceLower[2])/(dAlpha * Qref * totWingArea);
                double dCLdalphaTail = (tailForceUpper[2] - tailForceLower[2])/(dAlpha * Qref * totWingArea);

                //Adds distance from reference point as fraction of wing aero chord, multiplied by wing aero chord,
                //to the reference point
                double aeroCentreWingX = pointCOMs[1][0] - wingMeanAeroChord*dCMdalphaWing/dCLdalphaWing;
                double aeroCentreTailX = pointCOMs[1][0] - wingMeanAeroChord*dCMdalphaTail/dCLdalphaTail;

                tailLeverArm = aeroCentreWingX - aeroCentreTailX;
            }

            cout << "lever arm: " << tailLeverArm << endl;

            //Ensures that point is a stable rather than unstable equilibrium (M_alpha < 0)
            double dMdAlpha = (torqueUpper[1]+tailTorqueUpper[1]-torqueLower[1]-tailTorqueLower[1])/dAlpha;

            //Nondimensionalises forces
            double tailLiftCoeffLower = tailForceLower[2]/(Qref*tailHorizArea);
            double tailLiftCoeffUpper = tailForceUpper[2]/(Qref*tailHorizArea);

            //Gets change in tail angle of attack
            //Tail angle of attack formula
            //https://ocw.tudelft.nl/wp-content/uploads/Hand-out-Stability_01.pdf pg4
            //Downwash angle formula
            //https://www.sciencedirect.com/topics/engineering/downwash-angle
            double liftCoeffLower = forceLower[2]/(Qref*totWingArea);
            double liftCoeffUpper = forceUpper[2]/(Qref*totWingArea);
            double epsilonLower = 2.0*liftCoeffLower/(M_PI*wingAspectRatio);
            double epsilonUpper = 2.0*liftCoeffUpper/(M_PI*wingAspectRatio);
            double alphaTLower = alphaLower - epsilonLower;
            double alphaTUpper = alphaUpper - epsilonUpper;
            double dAlphaT = alphaTUpper - alphaTLower;

            //Derivative of tail lift coefficient with respect to tail angle of attack
            double dC_L_tdalphaT = (tailLiftCoeffUpper - tailLiftCoeffLower)/dAlphaT;
            
            double tailVolumeParameter = tailLeverArm*tailHorizArea/(wingMeanAeroChord*totWingArea);
            double Iyy = pointMOIs[1][1][1];

            //Approximates pitch damping
            double CMq = -2.0*tEfficiencyFactors[1]*(tailLeverArm/wingMeanAeroChord)*
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
            array<double, 3> currentConfig = {alpha, elevator, throttle};
            glm::dvec3 pointAeroForce = (aeroForces[1] + tailForces[1])*velocity*velocity;

            if(dMdAlpha < 0.0){
                double omega_n = sqrt(-dMdAlpha/Iyy);
                double zeta = -(Mq + MalphaDot)/(2*Iyy*omega_n );
                
                //Calls score function
                score = scoreFunc(currentConfig, velocity, pointAeroForce, omega_n, zeta, dMdAlpha, totalMass, paramNames, paramVals);
            }

            if(score > bestScore){
                bestScore = score;
                bestConfiguration = currentConfig;
            }

        }

    }else{
        cout << "No configuration found that enables trim flight on rank " << procRank << endl;
        bestScore = 0.0;
        bestConfiguration = {0.0, 0.0, 0.0};
    }

    cout << "Best Config, alpha: " << bestConfiguration[0] << ", elevator: " << bestConfiguration[1]
    << ", throttle: " << bestConfiguration[2] << endl;

    bestConfig = bestConfiguration;

    //Deletes folder
    string deleteDirBash = "rm -r Aerodynamics_Simulation_" + to_string(procRank);
    failure = system(deleteDirBash.c_str());
    if(failure) throw runtime_error("Failed to clean case directory " + to_string(procRank) + " after simulations");


    return bestScore;
}


//Gets relational matrix involved in translation of moment of inertia
glm::dmat3 aircraft::constructRelationMatrix(glm::dvec3 r) const{
    float dotP = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
    glm::dmat3 result = glm::dmat3(dotP) - glm::outerProduct(r, r);
    return result;
}

double aircraft::calculateVelocity(vector<extrusionData> extrusions, vector<int> motorParts, 
    double throttle, double dragCoeff, vector<glm::dvec3> motorThrustDirs, double alpha, 
    double yaw) const{

    //av^2 + bv + c = 0
    double a = -dragCoeff;
    double b = 0.0;
    double c = 0.0;

    //Sum force from each motor to get coefficients of quadratic equation involving velocity
    for(int m = 0; m < (int)motorParts.size(); m++){

        double maxThrust = extrusions[motorParts[m]].motor.maxThrust;
        double maxRPS = extrusions[motorParts[m]].motor.maxRPM/60.0;
        double propellorPitch = extrusions[motorParts[m]].motor.propPitch*0.025;

        //Find thrust and pitch speed based on linear model
        double thrust = 1.3*maxThrust*(throttle/100-0.23);
        double pitchSpeed = 0.7*maxRPS*propellorPitch*(throttle/100+0.42);
        
        //Get pitch and yaw of motor
        double motorPitch = alpha + atan(motorThrustDirs[m][2]/motorThrustDirs[m][0]);
        double motorYaw = yaw + atan(motorThrustDirs[m][1]/motorThrustDirs[m][0]);

        //Fraction of thrust in horizontal direction
        double horizThrust = thrust*cos(motorPitch)*cos(motorYaw);
        
        //Pitch speed is divided by cosines to account for reduced perpendicular
        //air velocity when at an angle
        b += horizThrust/(pitchSpeed/(cos(motorPitch)*cos(motorYaw)));
        c += -horizThrust;
    }

    //Solve quadratic equation. The two solutions are roughly centred around zero, with
    //one positive and one negative. Solve for the positive one
    double velocity = (-b+sqrt(b*b-4*a*c))/(2*a);

    return velocity;
}

//Finds values of volumetric mesh
void aircraft::getVolVals(const profile& partProfile, const extrusionData& extrusion, double density,
    double& mass, glm::dvec3& COM, glm::dmat3& MOI, glm::dmat2x3& boundingBox) const{
    
    //Gets extrusion
    vector<glm::dvec3> extrudePoints;
    vector<char> adjMatrix;
    int numTetras = generateExtrusion(partProfile, extrusion, adjMatrix, extrudePoints, boundingBox);

    //meshWindow window(500, 500);
    //window.draw3DSingle(extrudePoints, adjMatrix, 2.0);

    //Allocates an additional element due to how the tetrahedron searching code works
    //The vector in this space is unused
    vector<glm::ivec4> tetraIndices(numTetras+1);


    //Find tetrahedrons in adjacency matrix
    int adjSize = extrudePoints.size();
    int tetra = 0;
    //Only check upper right half of matrix to avoid double-counting tetrahedrons
    for(int r = 0; r < adjSize; r++){
        for(int c = r; c < adjSize; c++){
            //Check if there is an adjacency at the current location
            for(int p3 = c+1; p3 < adjSize*(int)adjMatrix[r*adjSize + c]; p3++){
                //Checks if there are two edges that connect both to eachother
                //and to opposite ends of the first edge that was found
                for(int p4 = p3+1; p4 < adjSize*(int)adjMatrix[r*adjSize + p3]*adjMatrix[p3*adjSize + c]; p4++){
                    glm::ivec4 newTetra(r, c, p3, p4);
                    tetraIndices[tetra] = newTetra;
                    //Only increments the index if ia tetrahedron was actually found
                    tetra += (int)adjMatrix[r*adjSize + p4]*adjMatrix[p4*adjSize + c]*adjMatrix[p4*adjSize + p3];
                }
            }
        }
    }


    //Use volumetric mesh to find relevant values
    float massSoFar = 0;
    glm::dvec3 COMSoFar(0.0);
    glm::dmat3 MOISoFar(0.0);
    int ind1[3] = {1, 0, 0};
    int ind2[3] = {2, 2, 1};


    for(int i = 0; i < numTetras; i++){

        //Vertices are not shifted and so MOI is around the origin
        glm::ivec4 verts = tetraIndices[i];
        //cout << verts[0] << ", " << verts[1] << ", " << verts[2] << ", " << verts[3] << endl;
        glm::dvec3 v1 = extrudePoints[verts[0]];
        glm::dvec3 v2 = extrudePoints[verts[1]];
        glm::dvec3 v3 = extrudePoints[verts[2]];
        glm::dvec3 v4 = extrudePoints[verts[3]];


        //Find mass
        glm::dmat3 jacobian = glm::dmat3(v2-v1, v3-v1, v4-v1);
        double tetraMass = density*abs(glm::determinant(jacobian))/6;

        //Find COM
        glm::dvec3 tetraCOM = 0.25*(v1 + v2 + v3 + v4);

        COMSoFar += tetraCOM*tetraMass;
        massSoFar += tetraMass;

        float xyzSums[3];
        float abcPrimes[3];
        //Calculates moment of inertia
        for(int j = 0; j < 3; j++){

            
            xyzSums[j] = v1[j]*v1[j] + v1[j]*v2[j] + v2[j]*v2[j] + v1[j]*v3[j] + v2[j]*v3[j] + 
                        v3[j]*v3[j] + v1[j]*v4[j] + v2[j]*v4[j] + v3[j]*v4[j] + v4[j]*v4[j];

            int i1 = ind1[j];
            int i2 = ind2[j];

            abcPrimes[j] = -0.5*(2*v1[i1]*v1[i2] + v1[i1]*v1[i2] + v1[i1]*v3[i2] + v1[i1]*v4[i2] +
                        v2[i1]*v1[i2] + 2*v2[i1]*v2[i2] + v2[i1]*v3[i2] + v2[i1]*v4[i2] +
                        v3[i1]*v1[i2] + v3[i1]*v2[i2] + 2*v3[i1]*v3[i2] + v3[i1]*v4[i2] +
                        v4[i1]*v1[i2] + v4[i1]*v2[i2] + v4[i1]*v3[i2] + 2*v4[i1]*v4[i2]);
        }

        MOISoFar += tetraMass*glm::dmat3(xyzSums[1] + xyzSums[2], abcPrimes[1], abcPrimes[2],
                                abcPrimes[1], xyzSums[0] + xyzSums[2], abcPrimes[0],
                                abcPrimes[2], abcPrimes[0], xyzSums[0] + xyzSums[1]);
    }


    mass = massSoFar + extrusion.pointMass;

    //Multiplies by common factor
    MOI = MOISoFar*0.1;

    //Adds point masses to calculation
    COM = (COMSoFar + extrusion.pointMass*extrusion.massLocation)/mass;
    MOI += extrusion.pointMass*constructRelationMatrix(extrusion.massLocation);;

}


void aircraft::getExtrusionData(vector<profile>& profiles, vector<extrusionData>& extrusions, 
    vector<string> paramNames, vector<double> paramVals, vector<int> discreteVals, double volMeshRes) const{

    //Gets profiles
    int numProfiles = profileFunctions.size();
    profiles.resize(numProfiles);
    for(int i = 0; i < numProfiles; i++){
        profile newProfile = profileFunctions[i] (paramNames, paramVals, volMeshRes);
        profiles[i] = newProfile;
    }

    
    //Gets extrusion data
    int numParts = partNames.size();
    extrusions.resize(numParts);
    for(int i = 0; i < numParts; i++){
        extrusions[i] = extrusionFunctions[i](paramNames, paramVals, volMeshRes);
    }

}


pair<vector<glm::dvec3>, vector<glm::dvec3>> aircraft::getAeroVals(vector<vector<double>> positionVariables, 
    const vector<double>& staticSDF, glm::ivec3 SDFSize, const vector<glm::dvec3>& XYZ, 
    const vector<profile>& profiles, vector<extrusionData> extrusions, vector<int> controlSurfaces,
    vector<glm::dvec3> controlAxes, vector<glm::dvec3> controlPivots, int horizontalStabiliser,
    int elevatorPart, const vector<double>& horizontalStabiliserSDF, vector<glm::dvec3> totalCOMs, 
    const vector<glm::dmat2x3>& boundingBoxes, glm::dmat2x3 totalBoundingBox, 
    vector<double>& tEfficiencyFactors, vector<glm::dvec3>& tailForces, 
    vector<glm::dvec3>& tailTorques, double surfMeshRes, int procRank, int nSimNodes, 
    int nSimTasksPerNode){

    vector<double> SDF = staticSDF;

    //Gets parent indices
    int numPositions = positionVariables.size();
    int numControl = controlSurfaces.size();

    vector<double> prevPosVals(numControl, 0.0);

    vector<glm::dvec3> netForces, netTorques;

    glm::dmat2x3 horizontalStabiliserBounds = boundingBoxes[horizontalStabiliser];

    //cout << numPositions << endl;

    string caseDir = "Aerodynamics_Simulation_" + to_string(procRank);

    //Makes a list of non-elevator control surfaces, as elevator is meshed separately 
    //with the horizontal stabiliser
    vector<int> flapSurfaces;
    for(int i = 0; i < numControl; i++){
        if(controlSurfaces[i] != elevatorPart){
            flapSurfaces.push_back(controlSurfaces[i]);
        }
    }

    for(int i = 0; i < numPositions; i++){
        vector<double> posVals = positionVariables[i];

        double pitch = posVals[0];
        double yaw = posVals[1];

        //Checks whether mesh needs to be regenerated
        int controlSameCount = 0;
        for(int c = 0; c < numControl; c++){
            if(prevPosVals[c] == posVals[c+2]) controlSameCount++;
        }


        if(controlSameCount < numControl || i == 0){

            vector<glm::dmat2x3> adjustedBoundingBoxes = boundingBoxes;


            //Assigns positions to control variables
            for(int c = 0; c < numControl; c++){
                int surfaceIndex = controlSurfaces[c];
                extrusions[surfaceIndex].rotateAngle = posVals[c+2];

                //cout << extrusions[surfaceIndex].rotateAngle << endl;

                //cout << "initial bounding box: " << adjustedBoundingBoxes[surfaceIndex][0][0] << ", " << adjustedBoundingBoxes[surfaceIndex][0][1] << ", " << adjustedBoundingBoxes[surfaceIndex][0][2] << " - "
                //    << adjustedBoundingBoxes[surfaceIndex][1][0] << ", " << adjustedBoundingBoxes[surfaceIndex][1][1] << ", " << adjustedBoundingBoxes[surfaceIndex][1][2] << endl;
        
                //cout << "initial total bounding box: " << adjustedTotBoundingBox[0][0] << ", " << adjustedTotBoundingBox[0][1] << ", " << adjustedTotBoundingBox[0][2] << " - "
                //    << adjustedTotBoundingBox[1][0] << ", " << adjustedTotBoundingBox[1][1] << ", " << adjustedTotBoundingBox[1][2] << endl;
                
                //Rotates bounding box
                glm::dquat rotation = glm::angleAxis(extrusions[surfaceIndex].rotateAngle, controlAxes[c]);
                //glm::dquat rotation = glm::angleAxis(0.0, controlAxes[c]);
                //cout << "axis: " << controlAxes[c][0] << ", " << controlAxes[c][1] << ", " << controlAxes[c][2] << endl;
                //cout << "pivot point: " << controlPivots[c][0] << ", " << controlPivots[c][1] << ", " << controlPivots[c][2] << endl;
                glm::dmat2x3 adjustedBounds;
                adjustedBounds[0] = rotation * (adjustedBoundingBoxes[surfaceIndex][0] - controlPivots[c]) + controlPivots[c];
                adjustedBounds[1] = rotation * (adjustedBoundingBoxes[surfaceIndex][1] - controlPivots[c]) + controlPivots[c];

                glm::dvec3 newMinBound = min(adjustedBounds[0], adjustedBounds[1]);
                glm::dvec3 newMaxBound = max(adjustedBounds[0], adjustedBounds[1]);

                adjustedBoundingBoxes[surfaceIndex] = glm::dmat2x3(newMinBound, newMaxBound);


                //cout << "rotated bounding box: " << adjustedBoundingBoxes[surfaceIndex][0][0] << ", " << adjustedBoundingBoxes[surfaceIndex][0][1] << ", " << adjustedBoundingBoxes[surfaceIndex][0][2] << " - "
                //    << adjustedBoundingBoxes[surfaceIndex][1][0] << ", " << adjustedBoundingBoxes[surfaceIndex][1][1] << ", " << adjustedBoundingBoxes[surfaceIndex][1][2] << endl;

                prevPosVals[c] = posVals[c+2];
            }

            //cout << "updating SDF" << endl;
            //Generates SDF
            vector<double> SDF = updateSDF(staticSDF, SDFSize, XYZ, profiles, partProfiles, extrusions, parentIndices,
                adjustedBoundingBoxes, totalBoundingBox, flapSurfaces, surfMeshRes);

            //Meshes SDF
            MC::mcMesh mesh;
            MC::marching_cube(SDF, SDFSize[0], SDFSize[1], SDFSize[2], mesh);
            glm::dvec3 minPoint = totalBoundingBox[0];
            double interval = 1.0/surfMeshRes;

            //Moves points from index coordinates to actual space coordinates
            for (int p = 0; p < (int)mesh.vertices.size(); p++){
                glm::dvec3 newPoint = minPoint + interval*glm::dvec3(mesh.vertices[p].x, mesh.vertices[p].y, mesh.vertices[p].z);
                mesh.vertices[p].x = newPoint[0];
                mesh.vertices[p].y = newPoint[1];
                mesh.vertices[p].z = newPoint[2];
            }
            
            writeMeshToObj(caseDir + "/aircraftMesh/aircraftModelRaw.obj", mesh);


            //Writes horizontal stabiliser to obj
            vector<double> stabiliserSDF = updateSDF(horizontalStabiliserSDF, SDFSize, XYZ, profiles, partProfiles, extrusions, parentIndices,
                adjustedBoundingBoxes, totalBoundingBox, {elevatorPart}, surfMeshRes);

            //Meshes SDF
            MC::mcMesh horizontalStabiliserMesh;
            MC::marching_cube(stabiliserSDF, SDFSize[0], SDFSize[1], SDFSize[2], horizontalStabiliserMesh);

            //Moves points from index coordinates to actual space coordinates
            for (int p = 0; p < (int)horizontalStabiliserMesh.vertices.size(); p++){
                glm::dvec3 newPoint = minPoint + 
                    interval*glm::dvec3(horizontalStabiliserMesh.vertices[p].x, 
                    horizontalStabiliserMesh.vertices[p].y, horizontalStabiliserMesh.vertices[p].z);

                horizontalStabiliserMesh.vertices[p].x = newPoint[0];
                horizontalStabiliserMesh.vertices[p].y = newPoint[1];
                horizontalStabiliserMesh.vertices[p].z = newPoint[2];
            }

            writeMeshToObj(caseDir + "/aircraftMesh/horizontalStabiliserRaw.obj", horizontalStabiliserMesh);
		

	    //MPI_Finalize();
	    //exit(0);

            glm::dmat2x3 widerAdjustedTotBoundingBox = totalBoundingBox;
            widerAdjustedTotBoundingBox[1] += glm::dvec3(0.2, 0.2, 0.2);
            widerAdjustedTotBoundingBox[0] += glm::dvec3(-1.7, -0.2, -0.2);

            cout << "Setting bounds on rank " << procRank << endl;
            
            //Writes correct bounding box
            string scriptCall = "./Aerodynamics_Simulation/updateBounds.sh " + to_string(widerAdjustedTotBoundingBox[0][0]) + " " +
                to_string(widerAdjustedTotBoundingBox[0][1]) + " " + to_string(widerAdjustedTotBoundingBox[0][2]) + " " +
                to_string(widerAdjustedTotBoundingBox[1][0]) + " " + to_string(widerAdjustedTotBoundingBox[1][1]) + " " +
                to_string(widerAdjustedTotBoundingBox[1][2]) + " " + 
                to_string(totalBoundingBox[0][0]) + " " + to_string(totalBoundingBox[0][1]) + " " + 
                to_string(totalBoundingBox[0][2]) + " " + to_string(totalBoundingBox[1][0]) + " " + 
                to_string(totalBoundingBox[1][1]) + " " + to_string(totalBoundingBox[1][2]) + " " +
                to_string(horizontalStabiliserBounds[0][0]) + " " + to_string(horizontalStabiliserBounds[0][1]) + " " +
                to_string(horizontalStabiliserBounds[0][2]) + " " + to_string(horizontalStabiliserBounds[1][0]) + " " +
                to_string(horizontalStabiliserBounds[1][1]) + " " + to_string(horizontalStabiliserBounds[1][2]) + " " +
                to_string(procRank);

            int failure = system(scriptCall.c_str());
            if(failure) throw std::runtime_error("Setting bounds failed in case " + to_string(procRank));

            cout << "Meshing on rank " << procRank << endl;

            scriptCall = "./" + caseDir + "/meshObj.sh " + to_string(procRank) + " " + 
                to_string(nSimNodes) + " " + to_string(nSimTasksPerNode);
            failure = system(scriptCall.c_str());
            if(failure) throw runtime_error("Meshing failed in case " + to_string(procRank));

            cout << "Completed meshing on rank " << procRank << endl;

            MPI_Barrier(MPI_COMM_WORLD);
            MPI_Finalize();
            exit(0);

        }



        //Gets turbulent dissipation rate
        //https://www.cfd-online.com/Wiki/Turbulence_free-stream_boundary_conditions
        double testVelocity = 40.0;
        double turbulenceIntensity = 0.003;
        double lengthScale = 0.5;
        double turbulentEnergy = 1.5*(testVelocity*turbulenceIntensity)*(testVelocity*turbulenceIntensity);
        double turbulentDissipationRate = pow(0.09, 0.75)*pow(turbulentEnergy, 1.5)/lengthScale;
        double specificTurbulenceDissipationRate = turbulentDissipationRate/(0.09*turbulentEnergy);

        glm::dvec3 velocity(-testVelocity*cos(pitch)*cos(yaw), testVelocity*sin(yaw)*sin(pitch), testVelocity*sin(pitch));

        
        //Sets boundary conditions based on velocity direciton
        string dictScriptCall = "./Aerodynamics_Simulation/updateDicts.sh " + to_string(velocity[0]) +
            " " + to_string(velocity[1]) + " " + to_string(velocity[2]) + " " + 
            to_string(roughnessHeight) + " " + to_string(specificTurbulenceDissipationRate) + " " +
            to_string(procRank);
        int failure = system(dictScriptCall.c_str());
        if(failure) throw std::runtime_error("Setting air velocity failed");


        //Sets lift and drag directions based on velocity direction
        glm::dvec3 normalisedVel = velocity/testVelocity;
        glm::dvec3 normalisedUp(sin(pitch), 0.0, cos(pitch));
        string forceScriptCall = "./Aerodynamics_Simulation/updateForces.sh " + to_string(totalCOMs[i][0]) +
            " " + to_string(totalCOMs[i][1]) + " " + to_string(totalCOMs[i][2]) + " " +
            to_string(normalisedUp[0]) + " " + to_string(normalisedUp[1]) + " " + to_string(normalisedUp[2]) +
            " " + to_string(normalisedVel[0]) + " " + to_string(normalisedVel[1]) + " " + to_string(normalisedVel[2]) +
            " " + to_string(testVelocity) + " " + to_string(procRank);
        failure = system(forceScriptCall.c_str());
        if(failure) throw std::runtime_error("Setting force details failed");

	    MPI_Finalize();
	    exit(0);

        //Runs simulation
        cout << "Running simulation on rank " << procRank << ", utilising " << nSimTasksPerNode <<
            " processes across " << nSimNodes << " nodes  with config AOA: " 
            << posVals[0] << ", elevator: " << posVals[2] << endl;
        double endTime = 0.3;
        double deltaT = 0.002;
        string simScriptCall = "./Aerodynamics_Simulation/runSim.sh " + to_string(endTime) + " " + to_string(deltaT) + " " +
            to_string(procRank) + " " + to_string(nSimNodes) + " " + to_string(nSimTasksPerNode);
        failure = system(simScriptCall.c_str());
        if(failure) throw std::runtime_error("Runnning simulation failed");

        

        //Get aerodynamic forces
        double tailUpstreamVelocity;
        pair<glm::dvec3, glm::dvec3> forceVals;
        glm::dvec3 tailForce, tailTorque;
        forceVals = getForces(caseDir + "/", tailUpstreamVelocity, tailForce, tailTorque, endTime);

        //Rotate forces back into correct position according to aircraft direction
        glm::dquat pitchRot = glm::angleAxis(-pitch, glm::dvec3(sin(yaw), -cos(yaw), 0.0));
        glm::dquat yawRot = glm::angleAxis(-yaw, normalisedUp);
        forceVals.first = yawRot * pitchRot * forceVals.first;
        forceVals.second = yawRot * pitchRot * forceVals.second;
        tailForce = yawRot * pitchRot * tailForce;
        tailTorque = yawRot * pitchRot * tailTorque;

        //Normalise for velocity
        forceVals.first /= (testVelocity*testVelocity);
        forceVals.second /= (testVelocity*testVelocity);

        //Gets tail efficiency factor
        double tEfficiencyFactor = (tailUpstreamVelocity/testVelocity)*(tailUpstreamVelocity/testVelocity);
        tEfficiencyFactors.push_back(tEfficiencyFactor);
        
        tailForce /= (testVelocity*testVelocity);
        tailTorque /= (testVelocity*testVelocity);
        tailForces.push_back(tailForce);
        tailTorques.push_back(tailTorque);

        netForces.push_back(forceVals.first);
        netTorques.push_back(forceVals.second);

        glm::dvec3 totalForce = forceVals.first + tailForce;
        glm::dvec3 totalTorque = forceVals.second + tailTorque;

        cout << "Force and torque from simulation " << i << " on rank " << procRank << ": " << 
            "(" << totalForce[0] + tailForce[0] << ", " << totalForce[1] + tailForce[1]<< ", " <<
            totalForce[2] + tailForce[2] << ") (" << totalTorque[0] + tailTorque[0] << ", " << 
            totalTorque[1] + tailTorque[1] << ", " << totalTorque[2] + tailTorque[2] << ")" << endl;
	
	MPI_Finalize();
	exit(0);
    }


    return make_pair(netForces, netTorques);
}

void aircraft::getPhysVals(vector<vector<double>> positionVariables, double staticMass,
    glm::dvec3 staticCOM, glm::dmat3 staticMOI, vector<double> controlMasses, 
    vector<glm::dvec3> controlPivots, vector<glm::dvec3> controlAxes, vector<glm::dvec3> controlCOMs,
    vector<glm::dmat3> controlMOIs, vector<glm::dvec3>& COMs, vector<glm::dmat3>& MOIs){

    int numPositions = positionVariables.size();
    int numControl = controlMasses.size();

    COMs.resize(numPositions);
    MOIs.resize(numPositions);
 
    for(int i = 0; i < numPositions; i++){
        vector<double> posVals = positionVariables[i];

        double totalMass = staticMass;
        glm::dvec3 totalCOM = staticCOM*totalMass;
        glm::dmat3 totalMOI = staticMOI;
        
        for(int c = 0; c < numControl; c++){

            glm::dquat controlRot = glm::angleAxis(posVals[c+2], controlAxes[c]);
            //Rotates COM
            glm::dvec3 controlCOM = controlRot * (controlCOMs[c] - controlPivots[c]) + controlPivots[c];
            totalCOM += controlCOM*controlMasses[c];

            //Rotates MOI
            glm::dmat3 controlMOI = controlMOIs[c];
            glm::dmat3 returnPivotMat = constructRelationMatrix(-1.0*controlPivots[c]);
            controlMOI += controlMasses[c]*returnPivotMat;
            glm::dmat3 rotMat = glm::mat3_cast(controlRot);
            controlMOI = rotMat*controlMOI*(glm::transpose(rotMat));
            returnPivotMat = constructRelationMatrix(controlPivots[c]);
            controlMOI = controlMOI + controlMasses[c]*returnPivotMat;

            totalMOI += controlMOI;
            totalMass += controlMasses[c];
        }

        totalCOM /= totalMass;

        //Moves the MOI so it is centred around the COM
        totalMOI += totalMass*constructRelationMatrix(-1.0*totalCOM);

        COMs[i] = totalCOM;
        MOIs[i] = totalMOI;

    }
}



/*void aircraft::plot(int SCREEN_WIDTH, int SCREEN_HEIGHT, vector<string> paramNames, vector<double> paramValues, vector<int> discreteVals, double volMeshRes){


    //Gets profiles
    vector<profile> profiles;

    int numParts = partNames.size();
    vector<extrusionData> extrusions;
    getExtrusionData(profiles, extrusions, paramNames, paramValues, discreteVals, volMeshRes);


    //Generates meshes
    vector<vector<glm::dvec3>> totalPoints(numParts);
    vector<vector<char>> adjMatrices;
    vector<char> adjMatrix(numParts);
    //Stores size of each adj matrix
    vector<int> adjSizes(numParts);


    for(int i = 0; i < numParts; i++){
        //Gets extrusion
        vector<glm::dvec3> points;
        glm::dmat2x3 boundingBox;
        generateExtrusion(profiles[partProfiles[i]], extrusions[i], adjMatrix, points, boundingBox);

        adjMatrices[i] = adjMatrix;

        int numPoints = points.size();

        adjSizes[i] = numPoints;

        totalPoints[i].resize(numPoints);

        for(int j = 0; j < numPoints; j++){
            totalPoints[i][j] = points[j];
        }

        //Applies parent transformations
        //Adds the part itself to list of transformations
        vector<int> transformIndices = {i};
        transformIndices.insert(transformIndices.begin() + 1, parentIndices[i].begin(), parentIndices[i].end());
        for(int p = 0; p < (int)transformIndices.size(); p++){
            //Apply transformations to part
            for(int j = 0; j < numPoints; j++){
                int tIndex = transformIndices[p];
                glm::dquat rotation = extrusions[tIndex].rotation;
                glm::dvec3 translation = extrusions[tIndex].translation;
                glm::dvec3 pivotPoint = extrusions[tIndex].pivotPoint;

                totalPoints[i][j] = rotation * (totalPoints[i][j] - pivotPoint) + translation + pivotPoint;
                
            }
        }
    }

    meshWindow window(SCREEN_WIDTH, SCREEN_HEIGHT);
    window.draw3D(totalPoints, adjMatrices, 1.5);
}*/

