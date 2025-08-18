#include "aircraft.h"


aircraft::aircraft(vector<string> _paramNames, function<void(vector<string>&, vector<double>&)> 
    _derivedParamsFunc,
    vector<function<profile(vector<string>, vector<double>, double)>> _profileFunctions){
    
    //Set class variables
    parameterNames = _paramNames;
    derivedParamsFunc = _derivedParamsFunc;
    profileFunctions = _profileFunctions;
}

void aircraft::addPart(string partName, double density, 
    function<extrusionData(vector<string>, vector<double>, double)>extrusionFunction, int profileIndex){
    //Adds first part to tree
    partNames.push_back(partName);
    partParents.push_back(-1);
    extrusionFunctions.push_back(extrusionFunction);
    partDensities.push_back(density);
    partProfiles.push_back(profileIndex);
}

void aircraft::addPart(string partName, string parentName, double density,
    function<extrusionData(vector<string>, vector<double>, double)> extrusionFunction, int profileIndex){

    //Gets index of parent
    int parentIndex = (int)(find(partNames.begin(), partNames.end(), parentName) - partNames.begin());
    //Checks that specified parent exists
    if(parentIndex == (int)partNames.size())
        throw std::runtime_error("Could not find part \"" + parentName + "\" in function aircraft::addPart");

    //Adds name of new part to list
    partNames.push_back(partName);
    
    //Adds parent index to list
    partParents.push_back(parentIndex);

    extrusionFunctions.push_back(extrusionFunction);
    partDensities.push_back(density);
    partProfiles.push_back(profileIndex);
}

vector<int> aircraft::findParents(int partIndex) const{
    int nextParent = partParents[partIndex];
    vector<int> parentList;

    while(nextParent > -1){
        parentList.push_back(nextParent);
        nextParent = partParents[nextParent];
    }

    return parentList;
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


void aircraft::calculateVals(vector<double> paramValues, double volMeshRes, double surfMeshRes,
    double &mass, glm::dvec3 &COM, glm::dmat3 &MOI){

        

    //Variables to store information about each extrusion
    vector<string> paramNames = parameterNames;

    //Gets derived parameter values. Values and names are inserted onto the end of argument vectors
    derivedParamsFunc(paramNames, paramValues);


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

    vector<glm::mat2x3> boundingBoxes;
    boundingBoxes.resize(numParts);
    glm::mat2x3 totalBoundingBox(-1e6, -1e6, -1e6, 1e6, 1e6, 1e6);

    for(int i = 0; i < numParts; i++){

        //Calculate variables based on profile and extrusion data
        glm::dvec3 partCOM;
        glm::dmat3 partMOI;
        double partVolume;
        int profileIndex = partProfiles[i];
        glm::mat2x3 boundingBox(1e6, 1e6, 1e6, -1e6, -1e6, -1e6);
        findVolVals(profiles[profileIndex], extrusions[i], partVolume, partCOM, partMOI, boundingBox);

        //Apply part's own transformations
        partCOM = extrusions[i].rotation * partCOM;
        partCOM += extrusions[i].translation;

        //Increases size of bounding box of each part by 5% in every direction


        
        double partMass = partVolume*partDensities[i];
        //Multiplies MOI by common factor
        MOI *= partMass;

        //Performs reverse operations on the MOI because these equations actually move the point
        //around which the part rotates, and so to move the part while continuing to rotate it around the 
        //origin
        glm::dmat3 rotMat = glm::mat3_cast(extrusions[i].rotation);
        partMOI = (glm::transpose(rotMat))*partMOI*rotMat;
        glm::dmat3 transMat = constructRelationMatrix(-1.0*extrusions[i].translation);
        partMOI = partMOI + partMass*transMat;

        //Applies transformations to bounding box
        boundingBox[0] = extrusions[i].rotation * glm::dvec3(boundingBox[0]);
        boundingBox[1] = extrusions[i].rotation * glm::dvec3(boundingBox[1]);
        boundingBox[0] = extrusions[i].translation + glm::dvec3(boundingBox[0]);
        boundingBox[1] = extrusions[i].translation + glm::dvec3(boundingBox[1]);

        //Get transformations applied to part
        vector<int> parentIndices = findParents(i);
        for(int p = 0; p < (int)parentIndices.size(); p++){
            int pIndex = parentIndices[p];
            //Apply transformations to part
            partCOM = extrusions[pIndex].rotation * partCOM;
            partCOM += extrusions[pIndex].translation;

            glm::dmat3 rotMat = glm::mat3_cast(extrusions[pIndex].rotation);
            partMOI = (glm::transpose(rotMat))*partMOI*rotMat;
            glm::dmat3 transMat = constructRelationMatrix(-1.0*extrusions[pIndex].translation);
            partMOI = partMOI + partMass*transMat;

            //Apply transformations to bounding box
            boundingBox[0] = extrusions[pIndex].rotation * glm::dvec3(boundingBox[0]);
            boundingBox[1] = extrusions[pIndex].rotation * glm::dvec3(boundingBox[1]);
            boundingBox[0] = extrusions[pIndex].translation + glm::dvec3(boundingBox[0]);
            boundingBox[1] = extrusions[pIndex].translation + glm::dvec3(boundingBox[1]);

        }

        //Adds 5% in every direction to bounding box
        float margin = 0.05;
        glm::vec3 boundSize = boundingBox[1] - boundingBox[0];
        boundingBox[0] -= boundSize*margin;
        boundingBox[1] += boundSize*margin;


        //Adjusts total bounding box if necessary
        totalBoundingBox[0][0] = min(totalBoundingBox[0][0], boundingBox[0][0]);
        totalBoundingBox[0][1] = min(totalBoundingBox[0][1], boundingBox[0][1]);
        totalBoundingBox[0][2] = min(totalBoundingBox[0][2], boundingBox[0][2]);
        totalBoundingBox[1][0] = max(totalBoundingBox[1][0], boundingBox[1][0]);
        totalBoundingBox[1][1] = max(totalBoundingBox[1][1], boundingBox[1][1]);
        totalBoundingBox[1][2] = max(totalBoundingBox[1][2], boundingBox[1][2]);

        boundingBoxes[i] = boundingBox;
        
        
        //Adds to total COM
        COMSoFar += partCOM*partMass;
        //Adds to total mass
        massSoFar += partMass;
    }

    //Gets COM
    COM = COMSoFar/massSoFar;

    //Translates point around which MOI is calculated so that it is around the COM of the assembly
    glm::dmat3 transMat = constructRelationMatrix(COM);
    MOI = MOISoFar + massSoFar*transMat;

    
    //Gets SDF
    vector<float> SDF;

    //Gets parents of each part
    vector<vector<int>> parentIndices;
    for(int i = 0; i < numParts; i++){
        parentIndices[i] = findParents(i);
    }

    //Gets SDF
    glm::ivec3 SDFSize = generateSDF(SDF, profiles, partProfiles, extrusions, parentIndices, totalBoundingBox, boundingBoxes, surfMeshRes);
    int SDFTotalSize = SDFSize[0] * SDFSize[1] * SDFSize[2];

    //Generates surface mesh
    MC::MC_FLOAT* field = new MC::MC_FLOAT[SDFTotalSize];

    #pragma omp simd
    for(int i = 0; i < SDFTotalSize; i++){
        field[i] = SDF[i];
    }

    MC::mcMesh surfaceMesh;
    MC::marching_cube(field, SDFSize[0], SDFSize[1], SDFSize[2], surfaceMesh);

    //Writes mesh to obj file
    writeMeshToObj("Aerodynamics/test.obj", surfaceMesh);


    //Gets aerodynamic forces



}

//Gets relational matrix involved in translation of moment of inertia
glm::dmat3 aircraft::constructRelationMatrix(glm::dvec3 r) const{
    float dotP = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
    glm::dmat3 result = glm::dmat3(dotP) - glm::outerProduct(r, r);
    return result;
}

//Finds values of volumetric mesh
void aircraft::findVolVals(const profile& partProfile, const extrusionData& extrusion, double& volume, 
    glm::dvec3& COM, glm::dmat3& MOI, glm::mat2x3& boundingBox) const{
    
    //Gets extrusion
    vector<glm::vec3> extrudePoints;
    vector<char> adjMatrix;
    int numTetras = generateExtrusion(partProfile, extrusion, adjMatrix, extrudePoints, boundingBox);

    //Allocates an additional element due to how the tetrahedron searching code works
    //The vector in this space is unused
    vector<glm::ivec4> tetraIndices;
    tetraIndices.resize(numTetras+1);


    //Find tetrahedrons in adjacency matrix
    int adjSize = extrudePoints.size();
    int tetra = 0;
    //Only check upper right half of matrix to avoid double-counting tetrahedrons
    //#pragma omp simd
    for(int r = 0; r < adjSize; r++){
        //#pragma omp simd
        for(int c = r; c < adjSize; c++){
            //Check if there is an adjacency at the current location
            //#pragma omp simd
            for(int p3 = c+1; p3 < adjSize*(int)adjMatrix[r*adjSize + c]; p3++){
                //Checks if there are two edges that connect both to eachother
                //and to opposite ends of the first edge that was found
                //#pragma omp simd reduction(+:tetra)
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
        #pragma omp simd
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


void aircraft::getExtrusionData(vector<profile>& profiles, vector<extrusionData>& extrusions, vector<double> paramValues, double volMeshRes) const{
    
    //Gets profiles
    int numProfiles = profileFunctions.size();
    for(int i = 0; i < numProfiles; i++){
        profile newProfile = (profileFunctions[i] (parameterNames, paramValues, volMeshRes));
        profiles[i] = newProfile;
    }


    int numParts = partNames.size();
    //Gets extrusion data
    for(int i = 0; i < numParts; i++){
        extrusions[i] = extrusionFunctions[i](parameterNames, paramValues, volMeshRes);
    }

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
    vector<vector<glm::vec3>> totalPoints;
    totalPoints.resize(numParts);
    vector<vector<char>> adjMatrices;
    vector<char> adjMatrix;
    adjMatrices.resize(numParts);
    //Stores size of each adj matrix
    vector<int> adjSizes;
    adjSizes.resize(numParts);

    for(int i = 0; i < numParts; i++){
        //Gets extrusion
        vector<glm::vec3> points;
        glm::mat2x3 boundingBox;
        generateExtrusion(profiles[partProfiles[i]], extrusions[i], adjMatrix, points, boundingBox);

        adjMatrices[i] = adjMatrix;

        int numPoints = points.size();

        adjSizes[i] = numPoints;

        totalPoints[i].resize(numPoints);


        //Applies initial transformation
        #pragma omp simd
        for(int j = 0; j < numPoints; j++){
            totalPoints[i][j] = extrusions[i].rotation * glm::dvec3(points[j]) + extrusions[i].translation;
        }

        //Applies parent transformations
        vector<int> parentIndices = findParents(i);
        for(int p = 0; p < (int)parentIndices.size(); p++){
            int pIndex = parentIndices[p];
            glm::quat rotation = extrusions[pIndex].rotation;
            glm::vec3 translation = extrusions[pIndex].translation;

            //Apply transformations to part
            #pragma omp simd
            for(int j = 0; j < numPoints; j++){
                totalPoints[i][j] = rotation * totalPoints[i][j] + translation;
            }

        }

    }

    meshWindow window(SCREEN_WIDTH, SCREEN_HEIGHT);
    window.draw3D(totalPoints, adjMatrices, 3.0f);

}

