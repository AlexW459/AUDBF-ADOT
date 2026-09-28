#include "getVolVals.h"

using namespace std;


//Finds values of volumetric mesh (MOI is around the origin)
void getVolVals(const extrusion& partExtrusion,
    vector<glm::dvec3> partPointMassLocations, vector<double> pointMasses, 
    double density, double& mass, glm::dvec3& COM, glm::dmat3& MOI){
    
    //Use volumetric mesh to find relevant values
    double massSoFar = 0;
    glm::dvec3 COMSoFar(0.0);
    glm::dmat3 MOISoFar(0.0);
    // Constants for MOI calculation
    int ind1[3] = {1, 0, 0};
    int ind2[3] = {2, 2, 1};

    int numTets = partExtrusion.tets.size();
    for(int i = 0; i < numTets; i++){

        //Vertices are not shifted and so MOI is around the origin
        glm::ivec4 verts = partExtrusion.tets[i];
        //cout << verts[0] << ", " << verts[1] << ", " << verts[2] << ", " << verts[3] << endl;
        glm::dvec3 v1 = partExtrusion.verts[verts[0]];
        glm::dvec3 v2 = partExtrusion.verts[verts[1]];
        glm::dvec3 v3 = partExtrusion.verts[verts[2]];
        glm::dvec3 v4 = partExtrusion.verts[verts[3]];


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
    int numPointMasses = pointMasses.size();
    for(int i = 0; i < numPointMasses; i++){
        massSoFar += pointMasses[i];
        COMSoFar += partPointMassLocations[i]*pointMasses[i];
        glm::dvec3 r = partPointMassLocations[i];
        MOI += pointMasses[i]*(glm::dmat3(dot(r, r)) - glm::outerProduct(r, r));
    }

    
    COM = COMSoFar / massSoFar;
    mass = massSoFar;
}

