#include "aircraft.h"

#define MC_CPP_USE_DOUBLE_PRECISION
#define MC_IMPLEM_ENABLE
#include "Mesh_Generation/MC.h"

aircraft::aircraft(vector<string> _paramNames,  vector<glm::dvec2> _paramRanges, 
    vector<dataTable> _discreteTables, derParamFuncType _derivedParamsFunc,
    vector<function<profile(vector<string>, vector<double>, double)>> _profileFunctions, 
    vector<vector<double>> _positionVariables, scoreFuncType _scoreFunc, double _roughnessHeight){
    
    //Set class variables
    paramNames = _paramNames;
    paramRanges = _paramRanges;
    derivedParamsFunc = _derivedParamsFunc;
    profileFunctions = _profileFunctions;
    discreteTables = _discreteTables;
    scoreFunc = _scoreFunc;
    simPositionVariables = _positionVariables;
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
    array<double, 3>& bestConfig, vector<double> simParams, bool writeObjs, int meshParallelOpt, 
    int simParallelOpt, int procRank, int nSimNodes, int nSimTasksPerNode){

    if(procRank == 0) {
        string deleteDirBash = "rm -r -f Aerodynamics_Simulation_*" + to_string(procRank);
        int failure = system(deleteDirBash.c_str());
    	if(failure) throw runtime_error("Failed to clean old case directories on " + to_string(procRank) + " before simulations");

	    cout << "cleaned old directories" << endl;
    }

    double PROFILE_RESOLUTION = simParams[1];
    double SDF_RESOLUTION = simParams[2];

    MPI_Barrier(MPI_COMM_WORLD);

    //Creates a unique case directory
    string copyBash = "cp -r Aerodynamics_Simulation Aerodynamics_Simulation_" + to_string(procRank);
    int failure = system(copyBash.c_str());
    if(failure) throw runtime_error("Failed to copy simulation directory for case " + to_string(procRank));

    //cout << "made a copy!" << endl;

    //Gets derived parameter values. Values and names are inserted onto the end of argument vectors
    vector<string> fullParamNames = paramNames;
    vector<double> fullParamVals = paramVals;
    vector<glm::dmat2x3> velRegions;
    derivedParamsFunc(fullParamNames, fullParamVals, discreteTables, discreteVals, velRegions);


    //Gets extrusion information and finds relevant values
    int numParts = partNames.size();
    vector<profile> profiles;
    vector<extrusionData> extrusions;
    //cout << "Getting extrusion data" << endl;
    getExtrusionData(profiles, extrusions, fullParamNames, fullParamVals, discreteVals, PROFILE_RESOLUTION); 


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

    //Stores values relating to point masses and directions
    vector<vector<glm::dvec3>> pointMassLocations;
    vector<vector<double>> pointMasses;
    vector<glm::dvec3> partDirections;

    //Stores force region parts to isolate
    vector<int> forceRegionParts;

    std::cout << "Getting vol vals on rank " << procRank << endl;
    for(int p = 0; p < numParts; p++){

        //Calculate variables based on profile and extrusion data
        glm::dvec3 partCOM;
        glm::dmat3 partMOI;
        double partMass = 0;
        int profileIndex = partProfiles[p];
        glm::dmat2x3 boundingBox(10.0, 10.0, 10.0, -10.0, -10.0, -10.0);
        getVolVals(profiles[profileIndex], extrusions[p], partDensities[p], partMass, partCOM, partMOI, boundingBox);

        //cout << "part bounding box: " << boundingBox[0][0] << ", " << boundingBox[0][1] << ", " << boundingBox[0][2] << " - "
        //    << boundingBox[1][0] << ", " << boundingBox[1][1] << ", " << boundingBox[1][2] << endl;

        //Get transformations applied to part
        vector<int> partParentIndices = parentIndices[p];
        //Adds part itself to list of transformations
        vector<int> transformIndices = {p};
        transformIndices.insert(transformIndices.begin() + 1, partParentIndices.begin(), partParentIndices.end());

        //If it is a control surface
        glm::dvec3 partPivot = extrusions[p].pivotPoint;
        glm::dvec3 partAxis = extrusions[p].controlAxis;

        //If it has a direction
        glm::dvec3 partDirection = extrusions[p].partDirection;

        //If it has point masses
        vector<glm::dvec3> partPointMassLocations = extrusions[p].massLocations;
        int numMasses = partPointMassLocations.size();

        //Applies transformations to part
        for(int t = 0; t < (int)transformIndices.size(); t++){
            int tIndex = transformIndices[t];

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

            // Apply transformations to part points and directions
            for(int i = 0; i < numMasses; i++){
                partPointMassLocations[i] -= extrusions[tIndex].pivotPoint;
                partPointMassLocations[i] = extrusions[tIndex].rotation * partPointMassLocations[i];
                partPointMassLocations[i] += extrusions[tIndex].translation + extrusions[tIndex].pivotPoint;
                partDirection = extrusions[tIndex].rotation * partDirection;
            }

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
        boundingBoxes[p] = boundingBox;

        massSoFar += partMass;

        // Adds point mass locations and part directions to vectors
        pointMassLocations[p] = partPointMassLocations;
        partDirections[p] = partDirection;

        if(glm::length(extrusions[p].controlAxis) == 0.0){
            //Adds to total COM
            COMSoFar += partCOM*partMass;
            //Adds to total mass
            staticMassSoFar += partMass;
            MOISoFar += partMOI;

            //Makes list of non-control surfaces
            staticSurfaces.push_back(p);

        }else{
            controlSurfaces.push_back(p);
            controlCOMs.push_back(partCOM*partMass);
            controlMOIs.push_back(partMOI);
            controlMasses.push_back(partMass);
            controlAxes.push_back(partAxis);
            controlPivotPoints.push_back(partPivot);

        }

        if(extrusions[p].isForceRegion){
            forceRegionParts.push_back(p);
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

    //Inits SDF
    vector<double> SDF;
    vector<glm::dvec3> XYZ;
    glm::ivec3 SDFSize = initSDF(SDF, XYZ, totalBoundingBox, SDF_RESOLUTION);
    //Adds static parts
    vector<double> staticSDF = updateSDF(SDF, SDFSize, XYZ, profiles, partProfiles, extrusions, parentIndices,
        boundingBoxes, totalBoundingBox, staticSurfaces, vector<double>(0.0, numParts), SDF_RESOLUTION);

    //Copies SDF without control surfaces for future use

    //applyGaussianBlur(0.2, 7, staticSDF, SDFSize);

    //cout << "ending process " << procRank << endl;
    //MPI_Finalize();
    //exit(0);

    vector<glm::dvec3> totalCOMs;
    vector<glm::dmat3> totalMOIs;

    //Use getPhysVals to find COMs and MOIs for each configuration
    getPhysVals(simPositionVariables, staticMass, staticCOM, staticMOI, controlMasses, 
        controlPivotPoints, controlAxes, controlCOMs, controlMOIs,totalCOMs, totalMOIs);


    cout << "Getting table of aerovals on rank " << procRank << endl;
    //Use getAeroVals to get force coefficients of each configuration (force divided by velocity squared)
    vector<vector<glm::dvec3>> regionForces, regionTorques;
    vector<vector<double>> regionVelMags;

    pair<vector<glm::dvec3>, vector<glm::dvec3>> aeroTable = getAeroVals(simPositionVariables, 
        staticSDF, SDFSize, XYZ, profiles, extrusions, controlSurfaces, controlAxes, 
        controlPivotPoints, forceRegionParts, velRegions, totalCOMs, boundingBoxes, 
        totalBoundingBox, regionForces, regionTorques, regionVelMags,
        simParams, writeObjs, procRank, meshParallelOpt, simParallelOpt, nSimNodes, nSimTasksPerNode);

    vector<glm::dvec3> totalAeroForces = aeroTable.first;
    vector<glm::dvec3> totalAeroTorques = aeroTable.second;

    double score = scoreFunc(fullParamNames, fullParamVals, discreteTables, discreteVals,
        simPositionVariables, totalMass, totalCOMs, totalMOIs, totalAeroForces, totalAeroTorques, 
        regionForces, regionTorques, regionVelMags, pointMassLocations, partDirections);

    //Deletes folder
    string deleteDirBash = "rm -r Aerodynamics_Simulation_" + to_string(procRank);
    failure = system(deleteDirBash.c_str());
    if(failure) throw runtime_error("Failed to clean case directory " + to_string(procRank) + " after simulations");


    return score;
}


//Gets relational matrix involved in translation of moment of inertia
glm::dmat3 aircraft::constructRelationMatrix(glm::dvec3 r) const{
    float dotP = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
    glm::dmat3 result = glm::dmat3(dotP) - glm::outerProduct(r, r);
    return result;
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
    double massSoFar = 0;
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

        double xyzSums[3];
        double abcPrimes[3];
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

    //Multiplies by common factor
    MOI = MOISoFar*0.1;

    //Adds point masses to calculation
    int numPointMasses = extrusion.pointMasses.size();
    for(int i = 0; i < numPointMasses; i++){
        massSoFar += extrusion.pointMasses[i];
        COMSoFar += extrusion.massLocations[i]*extrusion.pointMasses[i];
        MOI += extrusion.pointMasses[i]*constructRelationMatrix(extrusion.massLocations[i]);;
    }

    COM = COMSoFar / massSoFar;
}


void aircraft::getExtrusionData(vector<profile>& profiles, vector<extrusionData>& extrusions, 
    vector<string> paramNames, vector<double> paramVals, vector<int> discreteVals, double profileRes) const{

    //Gets profiles
    int numProfiles = profileFunctions.size();
    profiles.resize(numProfiles);
    for(int i = 0; i < numProfiles; i++){
        profile newProfile = profileFunctions[i] (paramNames, paramVals, profileRes);
        profiles[i] = newProfile;
    }

    
    //Gets extrusion data
    int numParts = partNames.size();
    extrusions.resize(numParts);
    for(int i = 0; i < numParts; i++){
        extrusions[i] = extrusionFunctions[i](paramNames, paramVals, profileRes);
    }

}


pair<vector<glm::dvec3>, vector<glm::dvec3>> aircraft::getAeroVals(vector<vector<double>> positionVariables, 
            const vector<double>& staticSDF, glm::ivec3 SDFSize, const vector<glm::dvec3>& XYZ,
            const vector<profile>& profiles, vector<extrusionData> extrusions, vector<int> controlSurfaces,
            vector<glm::dvec3> controlAxes, vector<glm::dvec3> controlPivots, vector<int> forceRegionParts,
            vector<glm::dmat2x3> velRegions, vector<glm::dvec3> totalCOMs, const vector<glm::dmat2x3>& boundingBoxes, 
            glm::dmat2x3 totalBoundingBox, vector<vector<glm::dvec3>>& regionForces, 
            vector<vector<glm::dvec3>>& regionTorques, vector<vector<double>>& regionAvgVels, 
            vector<double> simParams, bool writeObjs, int procRank, int meshParallelOpt, int simParallelOpt,
            int nSimNodes, int nSimTasksPerNode){

    vector<double> SDF = staticSDF;
    
    double SDF_RESOLUTION = simParams[2];

    int numParts = extrusions.size();
    int numPositions = positionVariables.size();
    int numControl = controlSurfaces.size();
    int numForceRegions = forceRegionParts.size();
    int numVelRegions = velRegions.size();

    vector<double> prevPosVals(numControl, 0.0);
    vector<double> controlAngles(numParts, 0.0);

    vector<glm::dvec3> totalForces, totalTorques;
    

    string caseDir = "Aerodynamics_Simulation_" + to_string(procRank);

    for(int i = 0; i < numPositions; i++){
        vector<double> posVals = positionVariables[i];
        

        glm::dvec3 flowVelocity(posVals[0], posVals[1], posVals[2]);

        //Checks whether mesh needs to be regenerated due to differeing control surface positions
        const int nPrecedingControl = 6;
        int controlSameCount = 0;
        for(int c = 0; c < numControl; c++){
            if(prevPosVals[c] == posVals[c+nPrecedingControl]) controlSameCount++;
            // Also adds control angles to vector
            int controlIndex = controlSurfaces[c];
            controlAngles[controlIndex] = posVals[c+nPrecedingControl];
        }


        if(controlSameCount < numControl || i == 0){

            vector<glm::dmat2x3> adjustedBoundingBoxes = boundingBoxes;


            //Assigns positions to control variables
            for(int c = 0; c < numControl; c++){
                int surfaceIndex = controlSurfaces[c];
                double controlAngle = posVals[c+nPrecedingControl];

                //cout << extrusions[surfaceIndex].rotateAngle << endl;

                //cout << "initial bounding box: " << adjustedBoundingBoxes[surfaceIndex][0][0] << ", " << adjustedBoundingBoxes[surfaceIndex][0][1] << ", " << adjustedBoundingBoxes[surfaceIndex][0][2] << " - "
                //    << adjustedBoundingBoxes[surfaceIndex][1][0] << ", " << adjustedBoundingBoxes[surfaceIndex][1][1] << ", " << adjustedBoundingBoxes[surfaceIndex][1][2] << endl;
        
                //cout << "initial total bounding box: " << adjustedTotBoundingBox[0][0] << ", " << adjustedTotBoundingBox[0][1] << ", " << adjustedTotBoundingBox[0][2] << " - "
                //    << adjustedTotBoundingBox[1][0] << ", " << adjustedTotBoundingBox[1][1] << ", " << adjustedTotBoundingBox[1][2] << endl;
                
                //Rotates bounding box
                glm::dquat rotation = glm::angleAxis(controlAngle, controlAxes[c]);
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

                // Update position of control surfaces
                prevPosVals[c-nPrecedingControl] = posVals[c];
            }

            //cout << "updating SDF" << endl;
            //Generates SDF
            vector<double> SDF = updateSDF(staticSDF, SDFSize, XYZ, profiles, partProfiles, extrusions, parentIndices,
                adjustedBoundingBoxes, totalBoundingBox, controlSurfaces, controlAngles, SDF_RESOLUTION);

            //Meshes SDF
            MC::mcMesh aircraftMesh;
            MC::marching_cube(SDF, SDFSize[0], SDFSize[1], SDFSize[2], aircraftMesh);
            glm::dvec3 minPoint = totalBoundingBox[0];
            double interval = 1.0/SDF_RESOLUTION;

            //Moves points from index coordinates to actual space coordinates
            for (int p = 0; p < (int)aircraftMesh.vertices.size(); p++){
                glm::dvec3 newPoint = minPoint + interval*glm::dvec3(aircraftMesh.vertices[p].x, 
                    aircraftMesh.vertices[p].y, aircraftMesh.vertices[p].z);
                aircraftMesh.vertices[p].x = newPoint[0];
                aircraftMesh.vertices[p].y = newPoint[1];
                aircraftMesh.vertices[p].z = newPoint[2];
            }
            
            writeMeshToObj(caseDir + "/aircraftMesh/aircraftModelRaw.obj", aircraftMesh);

            //Creates an obj file representing the current model
            if(writeObjs){
                writeMeshToObj("aircraftModel_config:" + to_string(i) + "_proc:" + to_string(procRank) + ".obj", aircraftMesh);
            }

            //Checks if user has asked for an early exit
            if(!RUN_OPTIMISATION_LOOP){

                #ifdef USE_SDL
                    SDL_Quit();
                #endif

                MPI_Barrier(MPI_COMM_WORLD);

                MPI_Finalize();
                exit(0);
            }



            glm::dmat2x3 widerAdjustedTotBoundingBox = totalBoundingBox;
            widerAdjustedTotBoundingBox[1] -= 0.01*glm::dvec3(flowVelocity[0], flowVelocity[1], flowVelocity[2]);
            widerAdjustedTotBoundingBox[0] += 0.01*glm::dvec3(flowVelocity[0], flowVelocity[1], flowVelocity[2]);
            //cout << "Setting bounds on rank " << procRank << endl;
            
            //Writes correct bounding box
            string boundScriptCall = "./Aerodynamics_Simulation/updateBounds.sh " + to_string(widerAdjustedTotBoundingBox[0][0]) + " " +
                to_string(widerAdjustedTotBoundingBox[0][1]) + " " + to_string(widerAdjustedTotBoundingBox[0][2]) + " " +
                to_string(widerAdjustedTotBoundingBox[1][0]) + " " + to_string(widerAdjustedTotBoundingBox[1][1]) + " " +
                to_string(widerAdjustedTotBoundingBox[1][2]) + " " +
                to_string(totalBoundingBox[0][0]) + " " + to_string(totalBoundingBox[0][1]) + " " + 
                to_string(totalBoundingBox[0][2]) + " " + to_string(totalBoundingBox[1][0]) + " " + 
                to_string(totalBoundingBox[1][1]) + " " + to_string(totalBoundingBox[1][2]) + " " +
                to_string(numForceRegions) + " ";
                
            // Adds force bounds
            for (int f = 0; f < numForceRegions; f++){
                int forceRegionPart = forceRegionParts[f];
                glm::dmat2x3 forceRegion = adjustedBoundingBoxes[forceRegionPart];
                string forceRegionBounds = to_string(forceRegion[0][0]) + " " + to_string(forceRegion[0][1]) + " " +
                to_string(forceRegion[0][2]) + " " + to_string(forceRegion[1][0]) + " " + to_string(forceRegion[1][1]) +
                " " + to_string(forceRegion[1][2]);
                boundScriptCall.append(forceRegionBounds);
            }

            // Adds velocity bounds
            boundScriptCall.append(" " + to_string(numVelRegions) + " ");
            for (int v = 0; v < numVelRegions; v++){
                glm::dmat2x3 velRegion = velRegions[v];
                string velRegionBounds = to_string(velRegion[0][0]) + " " + to_string(velRegion[0][1]) + " " +
                to_string(velRegion[0][2]) + " " + to_string(velRegion[1][0]) + " " + to_string(velRegion[1][1]) +
                " " + to_string(velRegion[1][2]);
                boundScriptCall.append(velRegionBounds);
            }

            int failure = system(boundScriptCall.c_str());
            if(failure) throw std::runtime_error("Setting bounds failed in case " + to_string(procRank));


            cout << "Meshing on rank " << procRank << endl;

            string meshScriptCall = "./Aerodynamics_Simulation/meshObj.sh " + to_string(procRank) + " " + 
                " \"" + OPENFOAM_SOURCE + "\" " + to_string(meshParallelOpt) + " " + to_string(nSimNodes) + 
                " " + to_string(nSimTasksPerNode);
            failure = system(meshScriptCall.c_str());
            if(failure) throw runtime_error("Meshing failed in case " + to_string(procRank));

            cout << "Completed meshing on rank " << procRank << endl;

            //MPI_Barrier(MPI_COMM_WORLD);
            //MPI_Finalize();
            //exit(0);
        }



        //Gets turbulent dissipation rate
        //https://www.cfd-online.com/Wiki/Turbulence_free-stream_boundary_conditions
        double flowVelocityMag = glm::length(flowVelocity);
        double TURBULENCE_INTENSITY = simParams[6];
        double TURBULENCE_LENGTH_SCALE = simParams[7];
        double turbulentEnergy = 1.5*(flowVelocityMag*TURBULENCE_INTENSITY)*(flowVelocityMag*TURBULENCE_INTENSITY);
        double turbulentDissipationRate = pow(0.09, 0.75)*pow(turbulentEnergy, 1.5)/TURBULENCE_LENGTH_SCALE;
        double specificTurbulenceDissipationRate = turbulentDissipationRate/(0.09*turbulentEnergy);

        //glm::dvec3 velocity(-1.0*testVelocity*cos(pitch)*cos(yaw), testVelocity*sin(yaw)*sin(pitch), testVelocity*sin(pitch));
        

        //Sets boundary conditions based on velocity direciton
        string dictScriptCall = "./Aerodynamics_Simulation/updateDicts.sh " + to_string(flowVelocity[0]) +
            " " + to_string(flowVelocity[1]) + " " + to_string(flowVelocity[2]) + " " + 
            to_string(roughnessHeight) + " " + to_string(specificTurbulenceDissipationRate) + " " +
            to_string(procRank);
        int failure = system(dictScriptCall.c_str());
        if(failure) throw std::runtime_error("Setting air velocity failed");


        //Sets COM, gravity direction, 
        glm::dvec3 gVec(posVals[4], posVals[5], posVals[6]);
        double RHO = simParams[5];
        string forceScriptCall = "./Aerodynamics_Simulation/updateForces.sh " + to_string(totalCOMs[i][0]) +
            " " + to_string(totalCOMs[i][1]) + " " + to_string(totalCOMs[i][2]) + " " +
            to_string(gVec[0]) + " " + to_string(gVec[1]) + " " + to_string(gVec[2]) +
            " " + to_string(flowVelocityMag) + " " + to_string(RHO) + " " + to_string(numForceRegions) + " " 
            + to_string(numVelRegions) + " " + to_string(procRank);
        failure = system(forceScriptCall.c_str());
        if(failure) throw std::runtime_error("Setting force details failed");


        //Runs simulation
        double endTime = simParams[3];
        double deltaT = simParams[4];
        string simScriptCall = "./Aerodynamics_Simulation/runSim.sh " + to_string(endTime) + " " + 
            to_string(deltaT) + " " + to_string(procRank) + " " + to_string(simParallelOpt) + " " 
            + to_string(nSimNodes) + " " + to_string(nSimTasksPerNode) + " \"" + OPENFOAM_SOURCE + "\"";
        failure = system(simScriptCall.c_str());
        if(failure) throw std::runtime_error("Runnning simulation failed");

        
        //Get aerodynamic forces
        pair<glm::dvec3, glm::dvec3> forceVals;
        forceVals = getForces(caseDir + "/", numForceRegions, numVelRegions, regionForces[i],
            regionTorques[i], regionAvgVels[i],  endTime);

        totalForces.push_back(forceVals.first);
        totalTorques.push_back(forceVals.second);

        
        /*cout << "Force and torque from simulation " << i << " on rank " << procRank << ": " << 
            "(" << forceVals.first[0] << ", " << forceVals.first[1] << ", " <<
            forceVals.first[2] << ") (" << forceVals.second[0] << ", " << 
            forceVals.second[1] << ", " << forceVals.second[2]] << ")" << endl;*/
	
	    MPI_Finalize();
	    exit(0);
    }


    return make_pair(totalForces, totalTorques);
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

            glm::dquat controlRot = glm::angleAxis(posVals[c+6], controlAxes[c]);
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


#ifdef USE_SDL
void aircraft::plot(int SCREEN_WIDTH, int SCREEN_HEIGHT, vector<string> paramNames, vector<double> paramValues, vector<int> discreteVals, double volMeshRes){


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
}
#endif
