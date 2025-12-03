#include "aircraft.h"

#define MC_CPP_USE_DOUBLE_PRECISION
#define MC_IMPLEM_ENABLE
#include "Mesh_Generation/MC.h"

aircraft::aircraft(vector<string> _paramNames, vector<dataTable> _discreteTables,
    function<void(vector<string>&, vector<double>&, const vector<dataTable>& discreteTables, 
    vector<int>)> _derivedParamsFunc,
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


void aircraft::calculateVals(vector<double> paramVals, vector<int> discreteVals, 
    double volMeshRes, double surfMeshRes, double &mass, vector<glm::dvec3> &COMs, 
    vector<glm::dmat3> &MOIs, vector<vector<double>> positionVariables){

    //Variables to store information required to construct extrusions
    //vector<string> paramNames = parameterNames;

    //Gets extrusion information and finds relevant values
    int numParts = partNames.size();
    vector<profile> profiles;
    vector<extrusionData> extrusions;
    getExtrusionData(profiles, extrusions, paramVals, discreteVals, volMeshRes); 

    glm::dvec3 COMSoFar(0.0);
    double massSoFar = 0.0;
    glm::dmat3 MOISoFar(0.0);

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

        glm::dvec3 partPivot(0.0, 0.0, 0.0);
        glm::dvec3 partAxis(0.0, 0.0, 0.0);

        if(extrusions[i].isControl){
            partPivot = extrusions[i].pivotPoint;
            partAxis = extrusions[i].controlAxis;
        }

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
    double staticMass = massSoFar;
    glm::dvec3 staticCOM = COMSoFar/massSoFar;
    glm::dmat3 staticMOI = MOISoFar;

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
    pair<vector<glm::dvec3>, vector<glm::dmat3>> physVals = getPhysVals(positionVariables, staticMass, staticCOM,
         staticMOI, controlMasses, controlPivotPoints, controlAxes, controlCOMs, controlMOIs);

    COMs = physVals.first;
    MOIs = physVals.second;

    vector<pair<glm::dvec3, glm::dvec3>> aeroTable = getAeroVals(positionVariables, staticSDF, SDFSize, XYZ, 
        profiles, extrusions, controlAxes, controlPivotPoints, COMs, boundingBoxes, totalBoundingBox, surfMeshRes);
    
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
    vector<double> paramVals, vector<int> discreteVals, double volMeshRes) const{

    //Gets derived parameter values. Values and names are inserted onto the end of argument vectors
    vector<string> paramNames = parameterNames;
    derivedParamsFunc(paramNames, paramVals, discreteTables, discreteVals);

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


vector<pair<glm::dvec3, glm::dvec3>> aircraft::getAeroVals(vector<vector<double>> positionVariables, 
    const vector<double>& staticSDF, glm::ivec3 SDFSize, const vector<glm::dvec3>& XYZ, 
    const vector<profile>& profiles, vector<extrusionData> extrusions,
    vector<glm::dvec3> controlAxes, vector<glm::dvec3> controlPivots, vector<glm::dvec3> totalCOMs, 
    const vector<glm::dmat2x3>& boundingBoxes, glm::dmat2x3 totalBoundingBox, double surfMeshRes){

    vector<double> SDF = staticSDF;

    //Gets parent indices
    int numPositions = positionVariables.size();
    int numControl = controlSurfaces.size();

    vector<double> prevPosVals;
    prevPosVals.resize(numControl, 0.0);


    vector<pair<glm::dvec3, glm::dvec3>> forceVals;
    forceVals.resize(numPositions);



    for(int i = 0; i < numPositions; i++){
        vector<double> posVals = positionVariables[i];

        double pitch = posVals[0];
        double yaw = posVals[1];

        //Checks whether mesh needs to be regenerated
        int controlSameCount = 0;
        for(int c = 0; c < numControl; c++){
            if(prevPosVals[c] == posVals[i+2]) controlSameCount++;
        }

        if(controlSameCount < numControl || i == 0){

            vector<glm::dmat2x3> adjustedBoundingBoxes = boundingBoxes;
            glm::dmat2x3 adjustedTotBoundingBox = totalBoundingBox;


            //Assigns positions to control variables
            for(int c = 0; c < numControl; c++){
                int surfaceIndex = controlSurfaces[c];
                extrusions[surfaceIndex].rotateAngle = posVals[c+2];

                //cout << "initial bounding box: " << boundingBox[0][0] << ", " << boundingBox[0][1] << ", " << boundingBox[0][2] << " - "
                //    << boundingBox[1][0] << ", " << boundingBox[1][1] << ", " << boundingBox[1][2] << endl;
        
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

                adjustedTotBoundingBox[0] = min(adjustedTotBoundingBox[0], newMinBound);
                adjustedTotBoundingBox[1] = max(adjustedTotBoundingBox[1], newMaxBound);


                //cout << "rotated total bounding box: " << adjustedTotBoundingBox[0][0] << ", " << adjustedTotBoundingBox[0][1] << ", " << adjustedTotBoundingBox[0][2] << " - "
                //    << adjustedTotBoundingBox[1][0] << ", " << adjustedTotBoundingBox[1][1] << ", " << adjustedTotBoundingBox[1][2] << endl;

                prevPosVals[c] = posVals[c+2];
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
            widerAdjustedTotBoundingBox[1] = adjustedTotBoundingBox[1] + glm::dvec3(0.5, 0.5, 0.5);
            widerAdjustedTotBoundingBox[0] = adjustedTotBoundingBox[0] + glm::dvec3(-2.0, -0.5, -0.5);

            //Writes correct bounding box to file

            string scriptCall = "./Aerodynamics_Simulation/updateBounds.sh " + to_string(widerAdjustedTotBoundingBox[0][0]) + " " +
                to_string(widerAdjustedTotBoundingBox[0][1]) + " " + to_string(widerAdjustedTotBoundingBox[0][2]) + " " +
                to_string(widerAdjustedTotBoundingBox[1][0]) + " " + to_string(widerAdjustedTotBoundingBox[1][1]) + " " +
                to_string(widerAdjustedTotBoundingBox[1][2]) + " " + 
                to_string(adjustedTotBoundingBox[0][0]) + " " + to_string(adjustedTotBoundingBox[0][1]) + " " + 
                to_string(adjustedTotBoundingBox[0][2]) + " " + to_string(adjustedTotBoundingBox[1][0]) + " " + 
                to_string(adjustedTotBoundingBox[1][1]) + " " + to_string(adjustedTotBoundingBox[1][2]);


            int success = system(scriptCall.c_str());
            if(success)throw std::runtime_error("Setting bounding box failed");

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
        int ySign = velocity[1] == 0 ? 0 : velocity[1] > 0 ? 1 :  -1;
        int zSign = velocity[2] == 0 ? 0 : velocity[2] > 0 ? 1 :  -1;


        string dictScriptCall = "./Aerodynamics_Simulation/updateDicts.sh " + to_string(velocity[0]) +
            " " + to_string(velocity[1]) + " " + to_string(velocity[2]) + " " + to_string(ySign) + "  " + to_string(zSign)
            + " " + to_string(roughnessHeight) + " " + to_string(specificTurbulenceDissipationRate);
        int success = system(dictScriptCall.c_str());
        if(success) throw std::runtime_error("Setting air velocity failed");

        glm::dvec3 normalisedVel = velocity/testVelocity;
        glm::dvec3 normalisedUp(sin(pitch)*cos(yaw), sin(yaw)*sin(pitch), cos(pitch));
        string forceScriptCall = "./Aerodynamics_Simulation/updateForces.sh " + to_string(totalCOMs[i][0]) +
            " " + to_string(totalCOMs[i][1]) + " " + to_string(totalCOMs[i][2]) + " " +
            to_string(normalisedUp[0]) + " " + to_string(normalisedUp[1]) + " " + to_string(normalisedUp[2]) +
            " " + to_string(normalisedVel[0]) + " " + to_string(normalisedVel[1]) + " " + to_string(normalisedVel[2]) +
            " " + to_string(testVelocity);
        success = system(forceScriptCall.c_str());
        if(success) throw std::runtime_error("Setting force details failed");

        //Runs simulation
        success = system("./Aerodynamics_Simulation/runSim.sh");
        if(success) throw std::runtime_error("Aerodynamics simulation failed");

        //Get aerodynamic forces
        forceVals[i] = getForces("Aerodynamics_Simulation/", "0.4");

    }

    return forceVals;
}

pair<vector<glm::dvec3>, vector<glm::dmat3>> aircraft::getPhysVals(vector<vector<double>> positionVariables,
    double staticMass, glm::dvec3 staticCOM, glm::dmat3 staticMOI, vector<double> controlMasses, 
    vector<glm::dvec3> controlPivots, vector<glm::dvec3> controlAxes, vector<glm::dvec3> controlCOMs,
    vector<glm::dmat3> controlMOIs){

    int numPositions = positionVariables.size();
    int numControl = controlMasses.size();

    pair<vector<glm::dvec3>, vector<glm::dmat3>> outputVals;
    outputVals.first.resize(numPositions);
    outputVals.second.resize(numPositions);
        
    
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

        outputVals.first[i] = totalCOM;
        outputVals.second[i] = totalMOI;

    }

    return outputVals;
}



void aircraft::plot(int SCREEN_WIDTH, int SCREEN_HEIGHT, vector<double> paramValues, vector<int> discreteVals, double volMeshRes){


    //Gets profiles
    int numProfiles = profileFunctions.size();
    vector<profile> profiles;
    profiles.resize(numProfiles);

    int numParts = partNames.size();
    vector<extrusionData> extrusions;
    extrusions.resize(numParts);
    getExtrusionData(profiles, extrusions, paramValues, discreteVals, volMeshRes);


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
    window.draw3D(totalPoints, adjMatrices, 1.5);

}

