#include "calculateScore.h"


using namespace std;

testModel ScoreEvaluator::model;
std::map<std::string, double> ScoreEvaluator::simParams;
bool ScoreEvaluator::writeObjs;
int ScoreEvaluator::procRank;
int ScoreEvaluator::nSimNodes;
int ScoreEvaluator::simParallelOpt;
int ScoreEvaluator::nSimTasksPerNode;  
int ScoreEvaluator::modelNum;
int ScoreEvaluator::nProcs;

void ScoreEvaluator::setSimParams(testModel _model, std::map<std::string, double> _simParams, 
    bool _writeObjs, int _nSimNodes, int _simParallelOpt, int _nSimTasksPerNode, int _procRank,
    int _nProcs){

    model = _model;
    simParams = _simParams;
    writeObjs = _writeObjs;
    simParallelOpt = _simParallelOpt;
    nSimNodes = _nSimNodes;
    nSimTasksPerNode = _nSimTasksPerNode;
    modelNum = 0;
    procRank = _procRank;
    nProcs = _nProcs;
}

void ScoreEvaluator::dealloc(){

    model.scoreFunc = {};
    model.derivedParamsFunc = {};
    for(int i = 0; i < (int)model.extrusionFuncs.size(); i++){
        model.extrusionFuncs[i] = {};
    }
    for(int i = 0; i < (int)model.profileFuncs.size(); i++){
        model.profileFuncs[i] = {};
    }

}


double ScoreEvaluator::calculateScore(const optim::ColVec_t& paramVals, optim::ColVec_t *gradOut,
    void *optData){

    int nParams = paramVals.rows();
    vector<profileFuncType> profileFunctions = model.profileFuncs;
    vector<string> partNames = model.partNames;
    vector<extrusionFuncType> extrusionFunctions = model.extrusionFuncs;
    vector<int> partProfiles = model.partProfiles;
    vector<vector<int>> parentIndices = model.parentIndices;
    vector<double> partDensities = model.partDensities;
    std::vector<std::vector<double>> simPositionVariables = model.simPositionVariables;
    scoreFuncType scoreFunc = model.scoreFunc;
    int numPositions = simPositionVariables.size();

    bool IBM_SIMULATION = simParams["IBM_SIMULATION"];

    double PROFILE_RESOLUTION = simParams["PROFILE_RESOLUTION"];
    double SDF_RESOLUTION = simParams["SDF_RESOLUTION"];
    double SDF_BAND_WIDTH = simParams["SDF_BAND_WIDTH"];

    double H_PERCENT = simParams["FINITE_DIFFERENCE_H"];

    //Creates a unique case directory for each position and test
    string originalCaseDir;
    string caseDir;
    if(simParams["IBM_SIMULATION"]) {
        originalCaseDir = string(projectRoot) + "/Aerodynamics_Simulation_IBM";
        caseDir = "Aerodynamics_Simulation_IBM_";
    }
    else {
        originalCaseDir = string(projectRoot) + "/Aerodynamics_Simulation_BFM";
        caseDir = "Aerodynamics_Simulation_BFM_";
    }

    // Number of tests is just one for all processes aside from the head
    int nTests;
    if(procRank == 0){
        nTests = gradOut ? 1 + 2*nParams : 1;
    }else{
        nTests = 1;
    }
    

    // Stores allocation of each simulation. Outer vec is test num, inner vec is position num
    // value is the assigned process num. Only filled and used by the first process (0)
    vector<vector<double>> paramMatrix(nTests, vector<double>(nParams));
    vector<double> testScores(nTests);


    if(procRank == 0){

        // Fills parameter matrix with values

        // Fills first row of matrix with point values
        for(int i = 0; i < nParams; i++) paramMatrix[0][i] = paramVals[i];

        // Fills in other tests needed to calculate graident
        if(gradOut){
            for(int i = 0; i < nParams; i++){
                paramMatrix[2*i+1] = paramMatrix[0];
                paramMatrix[2*i+2] = paramMatrix[0];

                // Adds finite difference points
                double val = paramMatrix[0][i];
                double h = val*H_PERCENT/100.0;
                paramMatrix[2*i+1][i] = val - h;
                paramMatrix[2*i+2][i] = val + h;

            }
        }


        for(int pos = 0; pos < (int)model.simPositionVariables.size(); pos++){
            for(int test  = 0; test < nTests; test++){
                string copyBash = "cp -r " + originalCaseDir + " " + caseDir + to_string(pos) + "_" + to_string(test); 
                int failure = system(copyBash.c_str());
                if(failure) throw runtime_error("Failed to copy simulation directory for case " + to_string(pos) + "_" + to_string(test));
            }
        }

        cout << "Copied simulation files to current directory" << endl;


    }else{
        // Checks that rank is still needed
        //MPI_Recv(&exitFlag, 1, MPI_CXX_BOOL, 0, ENTER_CALC_FUNC, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
    }


    // Stores previous bounding boxes for each position
    vector<glm::dmat2x3> prevDomainSizes(numPositions, glm::dmat2x3(0.0));


    for(int test = 0; test < nTests; test+=nProcs){

        // Gets values from model
        vector<string> fullParamNames = model.paramNames;
        vector<double> fullParamVals(nParams);

        bool runTest;
        double score;
        if(procRank == 0){
            runTest = true;
            fullParamVals = paramMatrix[test];

            // Sends necessary data to other ranks
            int nRemainingTests = min(nTests - test, nProcs);
            bool trueVal = true;
            bool falseVal = false;
            for(int t = 1; t < nRemainingTests; t++){
                // Tells ranks to run tests
                MPI_Send(&trueVal, 1, MPI_CXX_BOOL, t, RUN_TEST, MPI_COMM_WORLD);

                // Send parameters
                MPI_Send(paramMatrix[test + t].data(), nParams, MPI_DOUBLE, t, DATA_TO_HELPER, MPI_COMM_WORLD);

                // Send bounding boxes
                for(int pos = 0; pos < numPositions; pos++){
                    MPI_Send(&(prevDomainSizes[pos][0]), 3, MPI_DOUBLE, t, DATA_TO_HELPER, MPI_COMM_WORLD);
                    MPI_Send(&(prevDomainSizes[pos][1]), 3, MPI_DOUBLE, t, DATA_TO_HELPER, MPI_COMM_WORLD);
                }

                // Send model number
                MPI_Send(&modelNum, 1, MPI_INT, t, DATA_TO_HELPER, MPI_COMM_WORLD);

                // Send test numbers
                int testNum = test + t;
                MPI_Send(&testNum, 1, MPI_INT, t, DATA_TO_HELPER, MPI_COMM_WORLD);
            }
            for(int t = nRemainingTests; t < nProcs; t++){
                // Tells ranks not to run tests
                MPI_Send(&falseVal, 1, MPI_CXX_BOOL, t, RUN_TEST, MPI_COMM_WORLD);
            }

        }else{

            // Gets whether to run test or not
            MPI_Recv(&runTest, 1, MPI_CXX_BOOL, 0, RUN_TEST, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            if(runTest){
                // Gets parameters from main
                MPI_Recv(fullParamVals.data(), nParams, MPI_DOUBLE, 0, DATA_TO_HELPER, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

                // Gets bounding boxes of previous cases
                for(int pos = 0; pos < numPositions; pos++){
                    MPI_Recv(&(prevDomainSizes[pos][0]), 3, MPI_DOUBLE, 0, DATA_TO_HELPER, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                    MPI_Recv(&(prevDomainSizes[pos][1]), 3, MPI_DOUBLE, 0, DATA_TO_HELPER, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                }

                // Gets model number
                MPI_Recv(&modelNum, 1, MPI_INT, 0, DATA_TO_HELPER, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

                // Gets test number
                MPI_Recv(&test, 1, MPI_INT, 0, DATA_TO_HELPER, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

            }
        }




        if(runTest){

        vector<glm::dmat2x3> velRegions;

        //Gets derived parameter values. Values and names are inserted onto the end of argument vectors
        model.derivedParamsFunc(fullParamNames, fullParamVals, model.discreteTables, velRegions);

        // Gets part proiles
        int numProfiles = profileFunctions.size();
        vector<profile> profiles(numProfiles);
        for(int i = 0; i < numProfiles; i++){
            profile newProfile = profileFunctions[i] (fullParamNames, fullParamVals, PROFILE_RESOLUTION);
            profiles[i] = newProfile;
        }


        // Gets extrusions
        int numParts = partNames.size();
        vector<extrusion> extrusions(numParts);
        vector<extrusionData> extrusionInfo(numParts);
        for(int i = 0; i < numParts; i++){
            extrusionInfo[i] = extrusionFunctions[i](fullParamNames, fullParamVals, PROFILE_RESOLUTION);
            extrusions[i] = extrusion(profiles[partProfiles[i]], extrusionInfo[i]);

            //glm::dmat2x3 boundingBox = extrusions[i].computeBoundingBox();
            //cout << "part bounding box: " << boundingBox[0][0] << ", " << boundingBox[0][1] << ", "
            //<< boundingBox[0][2] << ", " << boundingBox[1][0] << ", " << boundingBox[1][1] << ", "
            //<< boundingBox[1][2] << endl;
        }

        glm::dvec3 staticCOM(0.0);
        double staticMass = 0.0;
        double controlMass = 0.0;
        glm::dmat3 staticMOI(0.0);

        glm::dmat2x3 totalBoundingBox(INF, INF, INF, -INF, -INF, -INF);

        //Stores values relating to control surfaces
        vector<double> controlMasses;
        vector<glm::dvec3> controlPivotPoints;
        vector<glm::dvec3> controlAxes;
        vector<int> controlParts;
        vector<int> staticParts;
        vector<glm::dvec3> controlCOMs;
        vector<glm::dmat3> controlMOIs;

        //Stores values relating to point masses and directions
        vector<vector<glm::dvec3>> pointMassLocations(numParts);
        vector<vector<double>> pointMasses(numParts);
        vector<glm::dvec3> partDirections(numParts);

        //Stores force regions
        vector<glm::dmat2x3> forceRegions;

        cout << "Getting vol vals on rank " << procRank << endl;
        for(int p = 0; p < numParts; p++){

            //Get transformations applied to part
            vector<int> partParentIndices = parentIndices[p];
            //Adds part itself to list of transformations
            vector<int> transformIndices = {p};
            transformIndices.insert(transformIndices.begin() + 1, partParentIndices.begin(), partParentIndices.end());
            

            //If it is a control surface
            glm::dvec3 partPivot = extrusionInfo[p].pivotPoint;
            glm::dvec3 partAxis = extrusionInfo[p].controlAxis;
            bool isControl = glm::length(extrusionInfo[p].controlAxis) > 0.0;

            //If it has a direction
            glm::dvec3 partDirection = extrusionInfo[p].partDirection;

            //If it has point masses
            //cout << extrusionInfo[p].massLocations.size() << endl;
            vector<glm::dvec3> partPointMassLocations = extrusionInfo[p].massLocations;
            int numMasses = partPointMassLocations.size();


            //Applies transformations to part
            for(int t = 0; t < (int)transformIndices.size(); t++){
                int tIndex = transformIndices[t];
                glm::dvec3 pivotPoint = extrusionInfo[tIndex].pivotPoint;
                glm::dquat rotation = extrusionInfo[tIndex].rotation;
                glm::dvec3 translation = extrusionInfo[tIndex].translation;
                
                extrusions[p].translate(-pivotPoint);
                extrusions[p].rotate(rotation);
                extrusions[p].translate(translation + pivotPoint);

                //Apply transformations to pivot point and axis of rotation
                partPivot -= pivotPoint;
                partPivot = rotation * partPivot;
                partPivot += translation + pivotPoint;

                partAxis = rotation * partAxis;

                // Apply transformations to part point masses and directions
                for(int i = 0; i < numMasses; i++){
                    partPointMassLocations[i] -= pivotPoint;
                    partPointMassLocations[i] = rotation * partPointMassLocations[i];
                    partPointMassLocations[i] += translation + pivotPoint;
                    partDirection = rotation * partDirection;
                }

            }


            extrusions[p].computeNormals();

            //Calculate variables based on profile and extrusion data
            glm::dvec3 partCOM;
            glm::dmat3 partMOI;
            double partMass;
            glm::dmat2x3 boundingBox;
            if(!isControl) boundingBox = extrusions[p].computeBoundingBox();
            else boundingBox = extrusions[p].computeBoundingBox(partAxis, partPivot);

            //cout << "part bounding box: " << boundingBox[0][0] << ", " << boundingBox[0][1] << ", "
            //<< boundingBox[0][2] << ", " << boundingBox[1][0] << ", " << boundingBox[1][1] << ", "
            //<< boundingBox[1][2] << endl;

            getVolVals(extrusions[p], partPointMassLocations, extrusionInfo[p].pointMasses, 
                partDensities[p], partMass, partCOM, partMOI);

            if(extrusionInfo[p].isForceRegion) forceRegions.push_back(boundingBox);


            //Increases bounding box by width of SDF band
            boundingBox[0] -= SDF_BAND_WIDTH * glm::dvec3(1.05);
            boundingBox[1] += SDF_BAND_WIDTH * glm::dvec3(1.05);

            //Adjusts total bounding box if necessary
            totalBoundingBox[0] = min(totalBoundingBox[0], boundingBox[0]);
            totalBoundingBox[1] = max(totalBoundingBox[1], boundingBox[1]);

            //cout << "part bounding box: " << boundingBox[0][0] << ", " << boundingBox[0][1] << ", "
            //<< boundingBox[0][2] << ", " << boundingBox[1][0] << ", " << boundingBox[1][1] << ", "
            //<< boundingBox[1][2] << endl;

            // Adds point mass locations and part directions to vectors
            pointMassLocations[p] = partPointMassLocations;
            partDirections[p] = partDirection;

            if(!isControl){
                //Adds to total COM
                staticCOM += partCOM*partMass;
                //Adds to total mass
                staticMass += partMass;
                staticMOI += partMOI;

                //cout << staticParts.size() << endl;
                //Makes list of non-control surfaces
                staticParts.push_back(p);

            }else{
                controlParts.push_back(p);
                controlMasses.push_back(partMass);
                controlAxes.push_back(partAxis);
                controlPivotPoints.push_back(partPivot);
                controlMass += partMass;

                controlCOMs.push_back(partCOM);
                controlMOIs.push_back(partMOI);
            }


        }

        //Adds 10% in every direction to bounding box
        double margin = 0.1;
        glm::dvec3 boundSize = totalBoundingBox[1] - totalBoundingBox[0];
        totalBoundingBox[0] -= boundSize*margin;
        totalBoundingBox[1] += boundSize*margin;


        //Gets COM
        staticCOM /= staticMass;


        //Inits SDF, also adjusts bounding box slightly
        vector<double> SDF, xVals, yVals, zVals;
        glm::ivec3 SDFsize = initSDF(SDF, xVals, yVals, zVals, totalBoundingBox, SDF_RESOLUTION);
        int totalSDFsize = SDFsize[0]*SDFsize[1]*SDFsize[2];
        //cout << "total SDF size: " << totalSDFsize << endl;

        // Gets bounding box of total domain
        double CELL_GRADIENT = simParams["CELL_GRADIENT"];
        double boundMultiple = (simParams["BOUND_MULTIPLE"]-1.0)/2.0;
        glm::dmat2x3 widerBoundingBox(totalBoundingBox[0] - boundMultiple*boundSize, 
            totalBoundingBox[1] + boundMultiple*boundSize);
            
        double interval = boundSize[0]/((double)SDFsize[0]-1.0);
        glm::ivec3 extraCells = ceil(boundMultiple*boundSize/((CELL_GRADIENT+1.0)/2.0*interval));


        //MPI_Finalize();
        //exit(0);

        //Adds static parts
        for(int i = 0; i < (int)staticParts.size(); i++){
            int staticIndex = staticParts[i];
            vector<double> partSDF;
            genPartSDF(extrusions[staticIndex], xVals, yVals, zVals, SDF_BAND_WIDTH,
                partSDF);
            
            SDFunion(SDF, partSDF);
        }


        const vector<double> staticSDF = SDF;

        // Gets a point in the mesh needed for openfoam meshing
        glm::ivec4 tetIndices = extrusions[0].tets[0];
        glm::dvec3 pointInMesh = (extrusions[0].verts[tetIndices[0]] + extrusions[0].verts[tetIndices[1]]
            + extrusions[0].verts[tetIndices[2]] + extrusions[0].verts[tetIndices[3]])/4.0;


        vector<glm::dvec3> totalCOMs(numPositions);
        vector<glm::dmat3> totalMOIs(numPositions);

        cout << "Getting aerodynamic forces on rank " << procRank << endl;

        //Use getAeroVals to get force coefficients of each configuration (force divided by velocity squared)
        int numControl = controlParts.size();
        int numForceRegions = forceRegions.size();
        int numVelRegions = velRegions.size();
        vector<extrusion> rotatedControlParts(numControl);
        vector<vector<double>> controlSDFs(numControl, vector<double>(totalSDFsize));
        vector<glm::dvec3> aeroForcesList(numPositions);
        vector<glm::dvec3> aeroTorquesList(numPositions);
        vector<vector<glm::dvec3>> regionForces(numPositions, vector<glm::dvec3>(numForceRegions));
        vector<vector<glm::dvec3>> regionTorques(numPositions, vector<glm::dvec3>(numForceRegions));
        vector<vector<double>> regionVelMags(numPositions, vector<double>(numVelRegions));


        for(int pos = 0; pos < numPositions; pos++){
            glm::dvec3 flowVelocity(simPositionVariables[pos][0], simPositionVariables[pos][1], simPositionVariables[pos][2]);

            // Directory containing most recent fields for that position
            string basePosDir = simParams["IBM_SIMULATION"] ? "Aerodynamics_Simulation_IBM_" + to_string(pos) :
                "Aerodynamics_Simulation_BFM_" + to_string(pos);
            string testCaseDir = simParams["IBM_SIMULATION"] ? "Aerodynamics_Simulation_IBM_" + to_string(pos) + "_" +
                to_string(test) : "Aerodynamics_Simulation_BFM_" + to_string(pos) + "_" + to_string(test);


            //Checks whether mesh needs to be regenerated due to differeing control surface positions
            const int nPrecedingControl = 6;
            int numControlMoved = 0;
            vector<bool> controlMoved(numControl, true);
            if(pos != 0){
                for(int c = 0; c < numControl; c++){
                    if(simPositionVariables[pos-1][c+nPrecedingControl] != simPositionVariables[pos][c+nPrecedingControl]){
                        controlMoved[c] = true;
                        numControlMoved++;
                    }else controlMoved[c] = false;
                }
            }


            if(numControlMoved > 0 || pos == 0){
                // Reset COM and MOI and SDF
                totalCOMs[pos] = staticCOM*staticMass;
                totalMOIs[pos] = staticMOI;
                SDF = staticSDF;


                //int numControlMoved = controlMoved.size();
                for(int c = 0; c < numControl; c++){
                    // Gets index in list of all parts
                    int controlIndex = controlParts[c];

                    if(controlMoved[c]){
                        
                        //Rotate necessary parts
                        double controlAngle = simPositionVariables[pos][c+nPrecedingControl];
                        glm::dquat rot = glm::angleAxis(controlAngle, controlAxes[c]);


                        // Reset part to original position
                        rotatedControlParts[c] = extrusions[controlIndex];
                        // Apply rotation
                        rotatedControlParts[c].translate(-controlPivotPoints[c]);
                        rotatedControlParts[c].rotate(rot);
                        rotatedControlParts[c].translate(controlPivotPoints[c]);

                        rotatedControlParts[c].computeNormals();

                        // Get new physical values
                        glm::dvec3 partCOM;
                        glm::dmat3 partMOI;
                        double partMass = controlMasses[c];
                        getVolVals(rotatedControlParts[c], pointMassLocations[controlIndex], 
                            pointMasses[controlIndex], partDensities[controlIndex], partMass, partCOM, partMOI);
                        controlCOMs[c] = partCOM;
                        totalMOIs[pos] += partMOI;

                        //Regenerate control SDF
                        genPartSDF(rotatedControlParts[c], xVals, yVals, zVals, SDF_BAND_WIDTH, controlSDFs[c]);
                    }

                    SDFunion(SDF, controlSDFs[c]);

                    totalCOMs[pos] += controlMasses[c]*controlCOMs[c];
                    totalMOIs[pos] += controlMOIs[c];

                }

                totalCOMs[pos] /= (staticMass + controlMass);



                if(!IBM_SIMULATION || writeObjs){
                    // Writes mesh to obj

                    //Meshes SDF
                    MC::mcMesh modelMesh;
                    MC::marchingCubes(SDF, xVals.size(), yVals.size(), zVals.size(), modelMesh);
                    glm::dvec3 minPoint = totalBoundingBox[0];
                    double interval = xVals[1]-xVals[0];

                    //Moves points from index coordinates to actual space coordinates
                    for (int p = 0; p < (int)modelMesh.vertices.size(); p++){
                        modelMesh.vertices[p] = minPoint + interval*modelMesh.vertices[p];
                    }

                    if(writeObjs){
                        writeMeshToObj("testModel_rank" + to_string(procRank) + "_pos" + to_string(pos) + ".obj", modelMesh);

                        #ifdef USE_SDL
                            SDL_Quit();
                        #endif

                        MPI_Finalize();
                        exit(0);

                    }else{
                        writeMeshToObj(caseDir + "/testModelMesh/testModelRaw.obj", modelMesh);
                    }
                }

                
                if(!IBM_SIMULATION){
                    // Generates mesh including force and velocity regions
                    generateOFmesh(simPositionVariables[pos], totalBoundingBox, pointInMesh,
                        forceRegions, velRegions, pos, test);
                }else{
                    glm::dvec3 COMtoBound = min(abs(totalCOMs[pos]-totalBoundingBox[0]), abs(totalCOMs[pos]-totalBoundingBox[1]));
                    double boundingRadius = min(COMtoBound[0], min(COMtoBound[1], COMtoBound[2]));
                    encodeSDF(SDF, totalBoundingBox, widerBoundingBox, SDFsize, totalCOMs[pos], boundingRadius, 
                        extraCells, CELL_GRADIENT, pos, test);
                }


            }else{
                // No need to rotate parts as values have not changed from last simulation
                totalCOMs[pos] = totalCOMs[pos-1];
                totalMOIs[pos] = totalMOIs[pos-1];
            }

            if(!IBM_SIMULATION){

                int failure;

                //Sets COR, gravity direction, RHO (Also clears function objects)
                glm::dvec3 gVec(simPositionVariables[pos][3], simPositionVariables[pos][4], simPositionVariables[pos][5]);
                double RHO = simParams.at("RHO");
                double flowVelocityMag = glm::length(flowVelocity);
                string forceScriptCall =  string(projectRoot) + "/src/simScripts/BFM/updateForces.sh " + to_string(totalCOMs[pos][0]) +
                    " " + to_string(totalCOMs[pos][1]) + " " + to_string(totalCOMs[pos][2]) + " " +
                    to_string(gVec[0]) + " " + to_string(gVec[1]) + " " + to_string(gVec[2]) +
                    " " + to_string(flowVelocityMag) + " " + to_string(RHO) + " " + to_string(pos) + " " + to_string(test);
                failure = system(forceScriptCall.c_str());
                if(failure) throw std::runtime_error("Setting force details failed");


                // Adds force and velocity function objects

                // Adds force functions
                /*int numForceRegions = forceRegions.size();
                for (int f = 0; f < numForceRegions; f++){
                    string forceFunctionScriptCall = string(projectRoot) + "/src/simScripts/BFM/addForceFunction.sh" + " " + to_string(pos) + " " + 
                        to_string(f) + " " + to_string(RHO) + " " + to_string(totalCOMs[pos][0]) + " " + to_string(totalCOMs[pos][1]) + 
                        " " + to_string(totalCOMs[pos][2]) + " " + to_string(flowVelocityMag);
                    failure = system(forceFunctionScriptCall.c_str());
                    if(failure) throw runtime_error("Adding force function object failed in case " + to_string(pos));
                }

                // Adds velocity functions
                int numVelRegions = velRegions.size();
                for (int v = 0; v < numVelRegions; v++){
                    string velFunctionScriptCall = string(projectRoot) + "/src/simScripts/BFM/addVelFunction.sh" + " " + to_string(pos) 
                        + " " + to_string(v);
                    failure = system(velFunctionScriptCall.c_str());
                    if(failure) throw runtime_error("Adding velocity region " + to_string(v) + " failed in case " + to_string(pos));
                }*/

                string velocityScriptCall = string(projectRoot) + "/src/simScripts/BFM/updateVelocity.sh " + to_string(flowVelocity[0]) +
                    " " + to_string(flowVelocity[1]) + " " + to_string(flowVelocity[2]) + " " + 
                    " " + to_string(pos) + " " + to_string(test);
                failure = system(velocityScriptCall.c_str());
                if(failure) throw runtime_error("Setting air velocity failed on position " + to_string(pos));
                
                // Runs simulation
                glm::dmat2x3 aeroForces = getAeroValsBFM(forceRegions.size(), velRegions.size(),
                    regionForces[pos], regionTorques[pos], regionVelMags[pos], simParams, 
                    pos, test, modelNum == 0, simParallelOpt, nSimNodes, nSimTasksPerNode);
                aeroForcesList[pos] = aeroForces[0];
                aeroTorquesList[pos] = aeroForces[1];

            }else{

                int failure;

                glm::dvec3 gVec(simPositionVariables[pos][3], simPositionVariables[pos][4], simPositionVariables[pos][5]);
                string forceScriptCall =  string(projectRoot) + "/src/simScripts/IBM/updateForces.sh " +
                    to_string(gVec[0]) + " " + to_string(gVec[1]) + " " + to_string(gVec[2]) +
                    " " + to_string(pos) + " " + to_string(test);
                failure = system(forceScriptCall.c_str());
                if(failure) throw std::runtime_error("Setting force details failed");

                string velocityScriptCall = string(projectRoot) + "/src/simScripts/IBM/updateVelocity.sh "
                     + to_string(flowVelocity[0]) +
                    " " + to_string(flowVelocity[1]) + " " + to_string(flowVelocity[2]) + " " +
                    to_string(pos) + " " + to_string(test);
                failure = system(velocityScriptCall.c_str());
                if(failure) throw std::runtime_error("Setting velocity details failed");

                // Maps fields if this is not the first simulation
                if(modelNum != 0){
                    // If target bounds are smaller than source bounds, add to cutting patches
                    // all patches that are either cut or no longer present

                    // Finds in which directions the new bounds are smaller
                    bool cutNX = prevDomainSizes[pos][0][0] < widerBoundingBox[0][0];
                    bool cutPX = prevDomainSizes[pos][1][0] > widerBoundingBox[1][0];
                    bool cutNY = prevDomainSizes[pos][0][1] < widerBoundingBox[0][1];
                    bool cutPY = prevDomainSizes[pos][1][1] > widerBoundingBox[1][1];
                    bool cutNZ = prevDomainSizes[pos][0][2] < widerBoundingBox[0][2];
                    bool cutPZ = prevDomainSizes[pos][1][2] > widerBoundingBox[1][2];

                    double lastTime = modelNum < 2 ? simParams["SIMULATION_LENGTH_INITIAL"] : 
                        simParams["SIMULATION_LENGTH"];

                    string mapFieldsScriptCall = string(projectRoot) + "/src/simScripts/IBM/mapFields.sh "
                    + OPENFOAM_SOURCE + " " + to_string(pos) + " " + to_string(test) + " "
                    + to_string(cutNX) + " " +  to_string(cutPX) + " " + to_string(cutNY) + " " +
                    to_string(cutPY) + " " + to_string(cutNZ) + " " + to_string(cutPZ) + " " +
                    to_string(lastTime);

                    failure = system(mapFieldsScriptCall.c_str());
                    if(failure) throw std::runtime_error("Mapping fields failed"s);
                }

                // Runs simulation
                glm::dmat2x3 aeroForces = getAeroValsIBM(forceRegions.size(), velRegions.size(),
                    regionForces[pos], regionTorques[pos], regionVelMags[pos], simParams, 
                    pos, test, modelNum == 0, simParallelOpt, nSimNodes, nSimTasksPerNode);
                aeroForcesList[pos] = aeroForces[0];
                aeroTorquesList[pos] = aeroForces[1];

                //cout << "Force: " << aeroForces[0][0] << " " << aeroForces[0][1] << " " << aeroForces[0][2] << endl;
            }

            // Saves bounding boxes
            if(procRank == 0){
                prevDomainSizes[pos] = widerBoundingBox;
                modelNum++;
            }

        }

        double totalMass = staticMass + controlMass;
        score = scoreFunc(fullParamNames, fullParamVals,
            simPositionVariables, totalMass, totalCOMs, totalMOIs, aeroForcesList, aeroTorquesList, 
            regionForces, regionTorques, regionVelMags, pointMassLocations, partDirections);

        
        cout << "score on rank " + to_string(procRank) + ": " << score << endl;

        }

        MPI_Barrier(MPI_COMM_WORLD);

        if(procRank == 0){
            testScores[test] = score;

            // Gets test scores from other ranks
            int nRemainingTests = min(nTests - test, nProcs);
            for(int i = 1; i < nRemainingTests; i++){
                MPI_Recv(&(testScores[test + i]), 1, MPI_DOUBLE, i, DATA_TO_HEAD, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
            }

            cout << "Saving simulation data from iteration " << modelNum << " test " << test << endl;

            // Saves data from simulations
            for(int pos = 0; pos < numPositions; pos++){

                string basePosDir = simParams["IBM_SIMULATION"] ? "Aerodynamics_Simulation_IBM_" + to_string(pos) :
                    "Aerodynamics_Simulation_BFM_" + to_string(pos);
                string testCaseDir = simParams["IBM_SIMULATION"] ? "Aerodynamics_Simulation_IBM_" + to_string(pos) + "_" +
                    to_string(0) : "Aerodynamics_Simulation_BFM_" + to_string(pos) + "_" + to_string(0);

                int failure = false;

                //Copies results to base case dir
                string saveSimResults1 = "cp -r " + testCaseDir + "/0* " + basePosDir + "/";
                failure = failure || system(saveSimResults1.c_str());

                string saveSimResults2 = "cp -r " + testCaseDir + "/constant " + basePosDir + "/";
                failure = failure || system(saveSimResults2.c_str());

                string saveSimResults3 = "cp -r " + testCaseDir + "/system* " + basePosDir + "/";
                failure = failure || system(saveSimResults1.c_str());

                string saveSimResults4 = "rm -f " + basePosDir + "/*/As";
                failure = failure || system(saveSimResults1.c_str());

                if(failure) throw std::runtime_error("Saving simulation results failed");

            }

            // Tells other processes to reenter loop
            bool falseVal = false;
            for(int i = 0; i < nProcs; i++){
                MPI_Send(&falseVal, 1, MPI_CXX_BOOL, i, EXIT_PROGRAM, MPI_COMM_WORLD);
            }

        }else if (runTest){

            // Send score to head
            MPI_Send(&score, 1, MPI_DOUBLE, 0, DATA_TO_HEAD, MPI_COMM_WORLD);
        }
    }
    

    if(procRank == 0){

        // Clears old sim directories
        string deleteDirBash = simParams["IBM_SIMULATION"] ? "rm -r Aerodynamics_Simulation_IBM_*_*" :
            "rm -r Aerodynamics_Simulation_BFM_*_*";
        int failure = system(deleteDirBash.c_str());
        if(failure) throw runtime_error("Failed to clean case directories after simulations");

        // Assigns gradient values
        if(gradOut){
            for(int i = 0; i < nParams; i++){
                double lowerScore = testScores[2*i+1];
                double upperScore = testScores[2*i+2];

                double lowerParam = paramMatrix[2*i+1][i];
                double upperParam = paramMatrix[2*i+2][i];

                double parDerivative = (upperScore - lowerScore)/(upperParam - lowerParam);

                (*gradOut)[i] = parDerivative;
                //cout << "Partial derivative " << i << ": " << parDerivative << endl;
            }
        }

    }

    return testScores[0];
}

