#include "aircraft.h"


aircraft::aircraft(vector<string> _paramNames, function<void(vector<string>&, vector<double>&)> 
    _derivedParamsFunc,
    vector<function<profile(vector<string>, vector<double>, double)>> _profileFunctions){
    
    //Set class variables
    parameterNames = _paramNames;
    derivedParamsFunc = _derivedParamsFunc;
    profileFunctions = _profileFunctions;
}

void aircraft::addPart(string partName, float density, 
    function<extrusionData(vector<string>, vector<double>, double)>extrusionFunction, int profileIndex){
    //Adds first part to tree
    partNames.push_back(partName);
    partParents.push_back(-1);
    extrusionFunctions.push_back(extrusionFunction);
    partDensities.push_back(density);
    partProfiles.push_back(profileIndex);
}

void aircraft::addPart(string partName, string parentName, float density,
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

vector<int> aircraft::findParents(int partIndex){
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
    float &mass, glm::vec3 &COM, glm::mat3 &MOI){

        

    //Variables to store information about each extrusion
    vector<string> paramNames = parameterNames;

    //Gets derived parameter values. Values and names are inserted onto the end of argument vectors
    derivedParamsFunc(paramNames, paramValues);
    //Combines params and derived params into single vectors
    
    //Gets profiles
    int numProfiles = profileFunctions.size();
    vector<profile> profiles;
    profiles.resize(numProfiles);
    int numParts = partNames.size();
    vector<extrusionData> extrusions;
    extrusions.resize(numParts);

    getExtrusionData(profiles, extrusions, paramValues, volMeshRes); 

    
    //Gets extrusion information and finds relevant values
    glm::vec3 COMSoFar(0, 0, 0);
    float massSoFar;
    glm::mat3 MOISoFar(0);

    glm::mat2x3 totalBoundingBox(1e6, 1e6, 1e6, -1e6, -1e6, -1e6);

    for(int i = 0; i < numParts; i++){

        //Calculate variables based on profile and extrusion data
        glm::vec3 partCOM;
        glm::mat3 partMOI;
        float partVolume;
        int profileIndex = partProfiles[i];
        glm::mat2x3 boundingBox(1e6, 1e6, 1e6, -1e6, -1e6, -1e6);
        findVolVals(profiles[profileIndex], extrusions[i], partVolume, partCOM, partMOI, boundingBox);

        //Apply part's own transformations
        partCOM = extrusions[i].rotation * partCOM;
        partCOM += extrusions[i].translation;

        
        float partMass = partVolume*partDensities[i];
        //Multiplies MOI by common factor
        MOI *= partMass;

        //Performs reverse operations on the MOI because these equations actually move the point
        //around which the part rotates, and so to move the part while continuing to rotate it around the 
        //origin
        glm::mat3 rotMat = glm::mat3_cast(extrusions[i].rotation);
        partMOI = (glm::transpose(rotMat))*partMOI*rotMat;
        glm::mat3 transMat = constructRelationMatrix(-1.0f*extrusions[i].translation);
        partMOI = partMOI + partMass*transMat;

        //Applies transformations to bounding box
        boundingBox[0] = extrusions[i].rotation * boundingBox[0];
        boundingBox[1] = extrusions[i].rotation * boundingBox[1];
        boundingBox[0] = extrusions[i].translation + boundingBox[0];
        boundingBox[1] = extrusions[i].translation + boundingBox[1];

        //Get transformations applied to part
        vector<int> parentIndices = findParents(i);
        for(int p = 0; p < (int)parentIndices.size(); p++){
            int pIndex = parentIndices[p];
            //Apply transformations to part
            partCOM = extrusions[pIndex].rotation * partCOM;
            partCOM += extrusions[pIndex].translation;

            glm::mat3 rotMat = glm::mat3_cast(extrusions[pIndex].rotation);
            partMOI = (glm::transpose(rotMat))*partMOI*rotMat;
            glm::mat3 transMat = constructRelationMatrix(-1.0f*extrusions[pIndex].translation);
            partMOI = partMOI + partMass*transMat;

            //Apply transformations to bounding box
            boundingBox[0] = extrusions[pIndex].rotation * boundingBox[0];
            boundingBox[1] = extrusions[pIndex].rotation * boundingBox[1];
            boundingBox[0] = extrusions[pIndex].translation + boundingBox[0];
            boundingBox[1] = extrusions[pIndex].translation + boundingBox[1];

        }

        //Adjusts total bounding box if necessary
        totalBoundingBox[0][0] = min(totalBoundingBox[0][0], boundingBox[0][0]);
        totalBoundingBox[0][1] = min(totalBoundingBox[0][1], boundingBox[0][1]);
        totalBoundingBox[0][2] = min(totalBoundingBox[0][2], boundingBox[0][2]);
        totalBoundingBox[1][0] = max(totalBoundingBox[1][0], boundingBox[1][0]);
        totalBoundingBox[1][1] = max(totalBoundingBox[1][1], boundingBox[1][1]);
        totalBoundingBox[1][2] = max(totalBoundingBox[1][2], boundingBox[1][2]);
        
        
        //Adds to total COM
        COMSoFar += partCOM*partMass;
        //Adds to total mass
        massSoFar += partMass;
    }

    //Gets COM
    COM = COMSoFar/massSoFar;


    //Translates point around which MOI is calculated so that it is around the COM of the assembly
    glm::mat3 transMat = constructRelationMatrix(COM);
    MOI = MOISoFar + massSoFar*transMat;

}

//Gets relational matrix involved in translation of moment of inertia
glm::mat3 aircraft::constructRelationMatrix(glm::vec3 r) const{
    float dotP = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
    glm::mat3 result = glm::mat3(dotP) - glm::outerProduct(r, r);
    return result;
}

//Finds values of volumetric mesh
void aircraft::findVolVals(const profile& partProfile, const extrusionData& extrusion, float& volume, 
    glm::vec3& COM, glm::mat3& MOI, glm::mat2x3& boundingBox) const{
    
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
                    glm::vec4 newTetra(r, c, p3, p4);
                    tetraIndices[tetra] = newTetra;
                    //Only increments the index if ia tetrahedron was actually found
                    tetra += (int)adjMatrix[r*adjSize + p4]*adjMatrix[p4*adjSize + c]*adjMatrix[p4*adjSize + p3];
                }
            }
        }
    }


    //Use volumetric mesh to find relevant values
    vector<float> tetraVolumes;
    tetraVolumes.resize(numTetras);
    float volumeSoFar = 0;
    glm::vec3 COMSoFar(0, 0, 0);
    glm::mat3 MOISoFar(0);
    int ind1[3] = {1, 0, 0};
    int ind2[3] = {2, 2, 1};


    for(int i = 0; i < numTetras; i++){

        //Vertices are not shifted and so MOI is around the origin
        glm::ivec4 verts = tetraIndices[i];
        //cout << verts[0] << ", " << verts[1] << ", " << verts[2] << ", " << verts[3] << endl;
        glm::vec3 v1 = extrudePoints[verts[0]];
        glm::vec3 v2 = extrudePoints[verts[1]];
        glm::vec3 v3 = extrudePoints[verts[2]];
        glm::vec3 v4 = extrudePoints[verts[3]];


        //Find volume
        glm::mat3 jacobian = glm::mat3(v2-v1, v3-v1, v4-v1);
        tetraVolumes[i] = abs(glm::determinant(jacobian))/6;

        //Find COM
        glm::vec3 tetraCOM = 0.25f*(v1 + v2 + v3 + v4);

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

        MOISoFar += glm::mat3(xyzSums[1] + xyzSums[2], abcPrimes[1], abcPrimes[2],
                                abcPrimes[1], xyzSums[0] + xyzSums[2], abcPrimes[0],
                                abcPrimes[2], abcPrimes[0], xyzSums[0] + xyzSums[1]);
    }

    //Assigns values to output variables
    volume = volumeSoFar;
    COM = COMSoFar /= volume;

    //Multiplies by common factor. Now equal to MOI/mass
    MOI = MOISoFar*0.1f;

}

//Generates volumetric mesh
int aircraft::generateExtrusion(const profile& partProfile, const extrusionData& extrusion, 
    vector<char>& adjMatrix, vector<glm::vec3>& points, glm::mat2x3& boundingBox) const{

    int numProfiles = extrusion.zSampleVals.size();
    int profileSize = partProfile.vertexCoords.size();
    int outerSize = profileSize;


    if(partProfile.inset){
        profileSize *= 2;
    }

    points.resize(numProfiles*profileSize);

    vector<glm::vec2> newPoints;
    newPoints.resize(profileSize);


    float extrusionBegin = *min_element(extrusion.zSampleVals.begin(), extrusion.zSampleVals.end());

    float extrusionEnd = *max_element(extrusion.zSampleVals.begin(), extrusion.zSampleVals.end());
    float extrusionLength = extrusionEnd - extrusionBegin;

    boundingBox[0][2] = extrusionBegin;
    boundingBox[1][2] = extrusionEnd;



    //Finds positions of profiles
    for(int i = 0; i < numProfiles; i++){
        float zPos = extrusion.zSampleVals[i];
        int profileBegin = i*profileSize;

        float profileSweep = extrusion.sweep * (zPos/extrusionLength);
        float profileYShift = extrusion.yPosVals[i];
        float profileScale = extrusion.scaleVals[i];
        for(int j = 0; j < outerSize; j++){
            glm::vec3 newPoint(partProfile.vertexCoords[j] * profileScale, zPos);
            newPoint[1] += profileYShift;
            newPoint[0] += profileSweep;
            newPoints[j] = newPoint;
            points[profileBegin + j] = newPoint;

            

            //Updates bounding box
            boundingBox[0][0] = min(boundingBox[0][0], newPoint[0]);
            boundingBox[0][1] = min(boundingBox[0][1], newPoint[1]);
            boundingBox[1][0] = max(boundingBox[1][0], newPoint[0]);
            boundingBox[1][1] = max(boundingBox[1][1], newPoint[1]);
        }


        //Adds inset points
        vector<glm::vec2> insetPoints = partProfile.generateInset(newPoints);
        //Loop does not run if there is no inset
        for(int j = 0; j < profileSize-outerSize; j++){
            glm::vec3 newPoint(insetPoints[j] * profileScale, zPos);
            newPoint[1] += profileYShift;
            newPoint[0] += profileSweep;
            newPoints[j] = newPoint;
            points[profileBegin + outerSize + j] = newPoint;
        }
    }

    
    int adjSize = numProfiles*profileSize;
    vector<char> profileAdjMatrix = partProfile.adjacencyMatrix;


    //Initialises adjacency matrix
    adjMatrix.resize(adjSize*adjSize);
    for(int i = 0; i < adjSize; i++){
        //Fills with zeros initially
        #pragma omp simd
        for(int j = 0; j < adjSize; j++){
            adjMatrix[i*adjSize + j] = 0;
        }
    }


    //Adds profile adjacency matrix to adjacency matrix
    for(int p = 0; p < numProfiles; p++){
        int startI = p*profileSize;

        #pragma omp simd collapse(2)
        for(int i = 0; i < profileSize; i++){
            for(int j = i; j < profileSize; j ++){

                int ind1 = startI + i;
                int ind2 = startI + j;
                adjMatrix[ind1*adjSize + ind2] = profileAdjMatrix[i*profileSize + j];
                adjMatrix[ind2*adjSize + ind1] = profileAdjMatrix[i*profileSize + j];

            }
        }        
    }

    //Adds identity matrices to matrix
    #pragma omp simd
    for(int i = profileSize; i < profileSize*numProfiles; i++){
        adjMatrix[(i-profileSize)*adjSize + i] = 1;
        adjMatrix[i*adjSize + (i-profileSize)] = 1;
    }

    //Splits rectangles into triangles
    #pragma omp simd collapse(3)
    for(int p = 0; p < numProfiles-1; p++){
        for(int i = 0; i < profileSize; i++){
            for(int j = i+1; j < profileSize; j ++){
                int ind1 = p*profileSize + i;
                int ind2 = (p + 1)*profileSize + j;
                adjMatrix[ind1*adjSize + ind2] = profileAdjMatrix[i*profileSize + j];
                adjMatrix[ind2*adjSize + ind1] = profileAdjMatrix[i*profileSize + j];
            }
        }
    }


    //Find number of tetrahedrons from adjacency matrix
    int numTetras = 3*partProfile.numTriangles*(numProfiles-1);
    
    return numTetras;
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
        glm::mat2x3 boundingBox(1e6, 1e6, 1e6, -1e6, -1e6, -1e6);
        generateExtrusion(profiles[partProfiles[i]], extrusions[i], adjMatrix, points, boundingBox);

        adjMatrices[i] = adjMatrix;

        int numPoints = points.size();

        adjSizes[i] = numPoints;

        totalPoints[i].resize(numPoints);


        //Applies initial transformation
        #pragma omp simd
        for(int j = 0; j < numPoints; j++){
            totalPoints[i][j] = extrusions[i].rotation * points[j] + extrusions[i].translation;
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

