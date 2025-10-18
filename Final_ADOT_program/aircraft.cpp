#include "aircraft.h"

#define MC_CPP_USE_DOUBLE_PRECISION
#define MC_IMPLEM_ENABLE
#include "Mesh_Generation/MC.h"

aircraft::aircraft(vector<string> _paramNames, vector<dataTable> _discreteTables,
    function<void(vector<string>&, vector<double>&, const vector<dataTable>& discreteTables, 
    vector<int>)> _derivedParamsFunc,
    vector<function<profile(vector<string>, vector<double>, double)>> _profileFunctions){
    
    //Set class variables
    parameterNames = _paramNames;
    derivedParamsFunc = _derivedParamsFunc;
    profileFunctions = _profileFunctions;
    discreteTables = _discreteTables;
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

void aircraft::addPart(string partName, string parentName, bool controlSurface, double density,
    function<extrusionData(vector<string>, vector<double>, double)> extrusionFunction, int profileIndex){

    //Gets index of parent
    int parentIndex = (int)(find(partNames.begin(), partNames.end(), parentName) - partNames.begin());
    //Checks that specified parent exists
    if(parentIndex == (int)partNames.size())
        throw std::runtime_error("Could not find part \"" + parentName + "\" in function aircraft::addPart");

    //Adds name of new part to list
    partNames.push_back(partName);
    
    //Adds parent index to list
    int partIndex = partNames.size() - 1;

    if(controlSurface) controlSurfaces.push_back(partIndex);

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


void aircraft::calculateVals(vector<double> paramValues, vector<int> discreteVals, 
    double volMeshRes, double surfMeshRes, double &mass, glm::dvec3 &COM, glm::dmat3 &MOI){

    //Variables to store information required to construct extrusions
    vector<string> paramNames = parameterNames;

    //Gets derived parameter values. Values and names are inserted onto the end of argument vectors
    derivedParamsFunc(paramNames, paramValues, discreteTables, discreteVals);
    fullParamNames = paramNames;

    //Gets profiles
    int numProfiles = profileFunctions.size();
    vector<profile> profiles;
    profiles.resize(numProfiles);
    int numParts = partNames.size();
    vector<extrusionData> extrusions;
    extrusions.resize(numParts);


    getExtrusionData(profiles, extrusions, paramValues, volMeshRes); 


    //Gets extrusion information and finds relevant values
    glm::dvec3 COMSoFar(0, 0, 0);
    double massSoFar = 0;
    glm::dmat3 MOISoFar(0);

    vector<glm::dmat2x3> boundingBoxes;
    boundingBoxes.resize(numParts);
    glm::dmat2x3 totalBoundingBox(50.0, 50.0, 50.0, -50.0, -50.0, -50.0);

    //Stores values relating to control points
    vector<glm::dmat3> controlMOIs;
    vector<glm::dvec3> controlCOMs;
    vector<double> controlMasses;
    vector<glm::dvec3> controlPivotPoints;
    vector<glm::dvec3> controlAxes;
    vector<int> staticSurfaces;

    for(int i = 0; i < numParts; i++){

        //Calculate variables based on profile and extrusion data
        glm::dvec3 partCOM;
        glm::dmat3 partMOI;
        double partVolume;
        int profileIndex = partProfiles[i];
        glm::dmat2x3 boundingBox(10.0, 10.0, 10.0, -10.0, -10.0, -10.0);
        findVolVals(profiles[profileIndex], extrusions[i], partVolume, partCOM, partMOI, boundingBox);

        //cout << "part bounding box: " << boundingBox[0][0] << ", " << boundingBox[0][1] << ", " << boundingBox[0][2] << " - "
        //    << boundingBox[1][0] << ", " << boundingBox[1][1] << ", " << boundingBox[1][2] << endl;
    
        double partMass = partVolume*partDensities[i];
        //Multiplies MOI by common factor
        partMOI *= partMass;

        //Get transformations applied to part
        vector<int> partParentIndices = parentIndices[i];
        //Adds part itself to list of transformations
        vector<int> transformIndices = {i};
        transformIndices.insert(transformIndices.begin() + 1, partParentIndices.begin(), partParentIndices.end());

        glm::dvec3 partPivot(0.0, 0.0, 0.0);
        glm::dvec3 partAxis(0.0, 0.0, 0.0);

        if(extrusions[i].isControl){
            partPivot = extrusions[i].pivotPoint;
            partAxis = extrusions[i].controlAxis;
        }

        for(int p = 0; p < (int)transformIndices.size(); p++){
            int tIndex = transformIndices[p];
            //Performs reverse operations on the MOI because these equations actually move the point
            //around which the part rotates, and so to move the part while continuing to rotate it around the 
            //origin
            partCOM -= extrusions[tIndex].pivotPoint;
            partCOM = extrusions[tIndex].rotation * partCOM;
            partCOM += extrusions[tIndex].translation + extrusions[tIndex].pivotPoint;

            glm::dmat3 returnPivotMat = constructRelationMatrix(extrusions[tIndex].pivotPoint);
            partMOI = partMOI + partMass*returnPivotMat;
            glm::dmat3 rotMat = glm::mat3_cast(extrusions[tIndex].rotation);
            partMOI = rotMat*partMOI*(glm::transpose(rotMat));
            glm::dmat3 transMat = constructRelationMatrix(-1.0*(extrusions[tIndex].translation + 
                extrusions[tIndex].pivotPoint));
            partMOI = partMOI + partMass*transMat;

            //Apply transformations to pivot point and axis of rotation
            partPivot -= extrusions[tIndex].pivotPoint;
            partPivot = extrusions[tIndex].rotation * partPivot;
            partPivot += extrusions[tIndex].translation + extrusions[tIndex].pivotPoint;

            //partAxis -= extrusions[tIndex].pivotPoint;
            partAxis = extrusions[tIndex].rotation * partAxis;
            //partAxis += extrusions[tIndex].translation + extrusions[tIndex].pivotPoint;


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

        if(!extrusions[i].isControl){
            //Adds to total COM
            COMSoFar += partCOM*partMass;
            //Adds to total mass
            massSoFar += partMass;
            MOISoFar += partMOI;

            //Makes list of non-control surfaces
            staticSurfaces.push_back(i);
        }else{
            controlCOMs.push_back(partCOM*partMass);
            controlMOIs.push_back(partMOI);
            controlMasses.push_back(partMass);
            controlAxes.push_back(partAxis);
            controlPivotPoints.push_back(partPivot);
        }

    }


    //Gets COM
    COM = COMSoFar/massSoFar;

    //Translates point around which MOI is calculated so that it is around the COM of the assembly
    glm::dmat3 transMat = constructRelationMatrix(COM);
    MOI = MOISoFar + massSoFar*transMat;

    //Gets SDF
    //staticSurfaces.insert(staticSurfaces.end(), controlSurfaces.begin(), controlSurfaces.end());

    //Inits SDF
    vector<double> SDF;
    vector<glm::dvec3> XYZ;
    glm::ivec3 SDFSize = initSDF(SDF, XYZ, totalBoundingBox, surfMeshRes);
    //Adds parts
    vector<double> staticSDF = updateSDF(SDF, SDFSize, XYZ, profiles, partProfiles, extrusions, parentIndices,
        boundingBoxes, totalBoundingBox, staticSurfaces, surfMeshRes);

    //Copies SDF without control surfaces for future use

    //applyGaussianBlur(0.2, 7, staticSDF, SDFSize);

    //Gets aerodynamic forces
    vector<vector<double>> positionVariables = {{0.0, 0.0}};
    vector<pair<glm::dvec3, glm::dvec3>> aeroTable = getAeroVals(positionVariables, staticSDF, SDFSize, XYZ, 
        profiles, extrusions, controlPivotPoints, controlAxes,boundingBoxes, totalBoundingBox, surfMeshRes);
    
}

//Gets relational matrix involved in translation of moment of inertia
glm::dmat3 aircraft::constructRelationMatrix(glm::dvec3 r) const{
    float dotP = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
    glm::dmat3 result = glm::dmat3(dotP) - glm::outerProduct(r, r);
    return result;
}

//Finds values of volumetric mesh
void aircraft::findVolVals(const profile& partProfile, const extrusionData& extrusion, double& volume, 
    glm::dvec3& COM, glm::dmat3& MOI, glm::dmat2x3& boundingBox) const{
    
    //Gets extrusion
    vector<glm::dvec3> extrudePoints;
    vector<char> adjMatrix;
    int numTetras = generateExtrusion(partProfile, extrusion, adjMatrix, extrudePoints, boundingBox);

    //meshWindow window(500, 500);
    //window.draw3DSingle(extrudePoints, adjMatrix, 2.0);

    //Allocates an additional element due to how the tetrahedron searching code works
    //The vector in this space is unused
    vector<glm::ivec4> tetraIndices;
    tetraIndices.resize(numTetras+1);


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
    vector<double> tetraVolumes;
    tetraVolumes.resize(numTetras);
    float volumeSoFar = 0;
    glm::dvec3 COMSoFar(0, 0, 0);
    glm::dmat3 MOISoFar(0);
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


        //Find volume
        glm::dmat3 jacobian = glm::dmat3(v2-v1, v3-v1, v4-v1);
        tetraVolumes[i] = abs(glm::determinant(jacobian))/6;

        //Find COM
        glm::dvec3 tetraCOM = 0.25*(v1 + v2 + v3 + v4);

        COMSoFar = COMSoFar + tetraCOM*(tetraVolumes[i]);
        volumeSoFar += tetraVolumes[i];

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

        MOISoFar += glm::dmat3(xyzSums[1] + xyzSums[2], abcPrimes[1], abcPrimes[2],
                                abcPrimes[1], xyzSums[0] + xyzSums[2], abcPrimes[0],
                                abcPrimes[2], abcPrimes[0], xyzSums[0] + xyzSums[1]);
    }

    //Assigns values to output variables
    volume = volumeSoFar;
    COM = COMSoFar /= volume;

    //Multiplies by common factor. Now equal to MOI/mass
    MOI = MOISoFar*0.1;

}


void aircraft::getExtrusionData(vector<profile>& profiles, vector<extrusionData>& extrusions, 
                                vector<double> paramValues, double volMeshRes) const{

    //Gets profiles
    int numProfiles = profileFunctions.size();
    for(int i = 0; i < numProfiles; i++){
        profile newProfile = (profileFunctions[i] (fullParamNames, paramValues, volMeshRes));
        profiles[i] = newProfile;
    }

    int numParts = partNames.size();
    //Gets extrusion data
    for(int i = 0; i < numParts; i++){
        extrusions[i] = extrusionFunctions[i](fullParamNames, paramValues, volMeshRes);
    }

}


vector<pair<glm::dvec3, glm::dvec3>> aircraft::getAeroVals(vector<vector<double>> positionVariables, 
    const vector<double>& staticSDF, glm::ivec3 SDFSize, const vector<glm::dvec3>& XYZ, 
    const vector<profile>& profiles, vector<extrusionData> extrusions, vector<glm::dvec3> controlPivots, 
    vector<glm::dvec3> controlAxes, const vector<glm::dmat2x3>& boundingBoxes, glm::dmat2x3 totalBoundingBox, 
    double surfMeshRes){

    vector<double> SDF = staticSDF;

    //Gets parent indices

    int numPositions = positionVariables.size();
    int numControl = controlSurfaces.size();

    vector<pair<glm::dvec3, glm::dvec3>> forceVals;
    forceVals.resize(numPositions);

    double testVelocity = 40.0;
    for(int i = 0; i < numPositions; i++){
        vector<double> posVals = positionVariables[i];

        double pitch = posVals[0];
        double yaw = posVals[1];

        vector<glm::dmat2x3> adjustedBoundingBoxes = boundingBoxes;
        glm::dmat2x3 adjustedTotBoundingBox = totalBoundingBox;

        //Assigns positions to control variables
        for(int c = 0; c < numControl; c++){
            int surfaceIndex = controlSurfaces[c];
            extrusions[surfaceIndex].rotateAngle = posVals[c+2];

            glm::dmat2x3 boundingBox = adjustedBoundingBoxes[surfaceIndex];
            cout << "initial bounding box: " << boundingBox[0][0] << ", " << boundingBox[0][1] << ", " << boundingBox[0][2] << " - "
                << boundingBox[1][0] << ", " << boundingBox[1][1] << ", " << boundingBox[1][2] << endl;
    
            cout << "initial total bounding box: " << adjustedTotBoundingBox[0][0] << ", " << adjustedTotBoundingBox[0][1] << ", " << adjustedTotBoundingBox[0][2] << " - "
                << adjustedTotBoundingBox[1][0] << ", " << adjustedTotBoundingBox[1][1] << ", " << adjustedTotBoundingBox[1][2] << endl;
            
            //Rotates bounding box
            glm::dquat rotation = glm::angleAxis(extrusions[surfaceIndex].rotateAngle, controlAxes[c]);
            cout << "axis: " << controlAxes[c][0] << ", " << controlAxes[c][1] << ", " << controlAxes[c][2] << endl;
            glm::dmat2x3 adjustedBounds;
            adjustedBounds[0] = rotation * (boundingBoxes[surfaceIndex][0] - controlPivots[c]) + controlPivots[c];
            adjustedBounds[1] = rotation * (boundingBoxes[surfaceIndex][1] - controlPivots[c]) + controlPivots[c];

            glm::dvec3 newMinBound = min(adjustedBounds[0], adjustedBounds[1]);
            glm::dvec3 newMaxBound = max(adjustedBounds[0], adjustedBounds[1]);

            adjustedBoundingBoxes[surfaceIndex] = glm::dmat2x3(newMinBound, newMaxBound);

            adjustedTotBoundingBox[0] = min(adjustedTotBoundingBox[0], adjustedBoundingBoxes[surfaceIndex][0]);
            adjustedTotBoundingBox[1] = max(adjustedTotBoundingBox[1], adjustedBoundingBoxes[surfaceIndex][1]);

            boundingBox = adjustedBoundingBoxes[surfaceIndex];

            cout << "rotated bounding box: " << boundingBox[0][0] << ", " << boundingBox[0][1] << ", " << boundingBox[0][2] << " - "
                << boundingBox[1][0] << ", " << boundingBox[1][1] << ", " << boundingBox[1][2] << endl;
    
            cout << "rotated total bounding box: " << adjustedTotBoundingBox[0][0] << ", " << adjustedTotBoundingBox[0][1] << ", " << adjustedTotBoundingBox[0][2] << " - "
                << adjustedTotBoundingBox[1][0] << ", " << adjustedTotBoundingBox[1][1] << ", " << adjustedTotBoundingBox[1][2] << endl;
            
        }

        
        //Generates SDF
        vector<double> SDF = updateSDF(staticSDF, SDFSize, XYZ, profiles, partProfiles, extrusions, parentIndices,
            adjustedBoundingBoxes, adjustedTotBoundingBox, controlSurfaces, surfMeshRes);

        //Meshes SDF
        MC::mcMesh mesh;
        MC::marching_cube(SDF, SDFSize[0], SDFSize[1], SDFSize[2], mesh);
        glm::dvec3 minPoint = totalBoundingBox[0];
        double interval = 1/surfMeshRes;

        //Moves points from index coordinates to actual space coordinates
        for (int p = 0; p < (int)mesh.vertices.size(); p++){
            glm::dvec3 newPoint = minPoint + interval*glm::dvec3(mesh.vertices[p].x, mesh.vertices[p].y, mesh.vertices[p].z);
            mesh.vertices[p].x = newPoint[0];
            mesh.vertices[p].y = newPoint[1];
            mesh.vertices[p].z = newPoint[2];
        }
        
        writeMeshToObj("Aerodynamics_Simulation/aircraftMesh/aircraftModelRaw.obj", mesh);

        glm::dmat2x3 widerAdjustedTotBoundingBox;
        widerAdjustedTotBoundingBox[1] = adjustedTotBoundingBox[1] + glm::dvec3(0.2, 0.2, 0.2);
        widerAdjustedTotBoundingBox[0] = adjustedTotBoundingBox[0] + glm::dvec3(-1.0, -0.2, -0.2);

        //Writes correct bounding box to file
        glm::dvec3 velocity(-testVelocity*cos(pitch)*cos(yaw), -testVelocity*cos(yaw)*sin(pitch), -testVelocity*sin(pitch));
        int ySign = velocity[1] == 0 ? 0 : velocity[1] > 0 ? 1 :  -1;
        int zSign = velocity[2] == 0 ? 0 : velocity[2] > 0 ? 1 :  -1;


        string scriptCall = "./Aerodynamics_Simulation/updateDicts.sh " + to_string(widerAdjustedTotBoundingBox[0][0]) + " " +
            to_string(widerAdjustedTotBoundingBox[0][1]) + " " + to_string(widerAdjustedTotBoundingBox[0][2]) + " " +
            to_string(widerAdjustedTotBoundingBox[1][0]) + " " + to_string(widerAdjustedTotBoundingBox[1][1]) + " " +
            to_string(widerAdjustedTotBoundingBox[1][2]) + " " + to_string(velocity[0]) + " " + to_string(velocity[1]) +
            " " + to_string(velocity[2]) + " " + to_string(ySign) + " " + to_string(zSign) + " " + 
            to_string(adjustedTotBoundingBox[0][0]) + " " + to_string(adjustedTotBoundingBox[0][1]) + " " + 
            to_string(adjustedTotBoundingBox[0][2]) + " " + to_string(adjustedTotBoundingBox[1][0]) + " " + 
            to_string(adjustedTotBoundingBox[1][1]) + " " + to_string(adjustedTotBoundingBox[1][2]);


        int success = system(scriptCall.c_str());

        if(success){
            throw std::runtime_error("Flight simulation failed with error code " + success);
        }

        //Get aerodynamic forces
        forceVals[i].first = glm::dvec3(0.0, 0.0, 0.0);
        forceVals[i].second = glm::dvec3(0.0, 0.0, 0.0);

    }

    return forceVals;
}


void aircraft::plot(int SCREEN_WIDTH, int SCREEN_HEIGHT, vector<double> paramValues, double volMeshRes){
    //Gets profiles
    int numProfiles = profileFunctions.size();
    vector<profile> profiles;
    profiles.resize(numProfiles);
    int numParts = partNames.size();
    vector<extrusionData> extrusions;
    extrusions.resize(numParts);
    getExtrusionData(profiles, extrusions, paramValues, volMeshRes);



    //Generates meshes
    vector<vector<glm::dvec3>> totalPoints;
    totalPoints.resize(numParts);
    vector<vector<char>> adjMatrices;
    vector<char> adjMatrix;
    adjMatrices.resize(numParts);
    //Stores size of each adj matrix
    vector<int> adjSizes;
    adjSizes.resize(numParts);

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
    window.draw3D(totalPoints, adjMatrices, 1.5f);

}

