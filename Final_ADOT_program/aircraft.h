#pragma once

#include <vector>
#include <string>
#include <utility>
#define GLM_FORCE_DEFAULT_ALIGNED_GENTYPES
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <functional>
#include <mpi.h>

#include "Mesh_Generation/profile.h"
#include "Mesh_Generation/surfaceMeshGen.h"
#include "Mesh_Generation/extrusionGen.h"
#include "Aerodynamics_Simulation/getForces.h"
#include "readCSV.h"

#define RHO 1.225
#define G_CONSTANT 9.81

using namespace std;

enum HELPER_CMD {
    HELPER_QUIT,
    HELPER_MESH,
    HELPER_SIM
};


class aircraft{
    public:
        aircraft(vector<string> _paramNames, vector<dataTable> _discreteTables,
            function<void(vector<string>&, vector<double>&, const vector<dataTable>& discreteTables, 
            vector<int>, double&, double&, double&, double&)> _derivedParamsFunc, 
            vector<function<profile(vector<string>, vector<double>, double)>> _profileFunctions, 
            double _roughnessHeight);

        void addPart(string partName, double density,
            function<extrusionData(vector<string>, vector<double>, double)> extrusionFunction, int profileIndex);
        void addPart(string partName, string parentPart, double density,
            function<extrusionData(vector<string>, vector<double>, double)> extrusionFunction, int profileIndex);

        int findPart(string partName);

        //ScoreFunc parameters are: configuration variables (AOA, elevator, throttle), 
        //aerodynamic forces, velocity, oscillation frequency, damping coefficient, 
        //dMdalpha, mass, paramNames, paramVals
        double calculateScore(vector<double> paramVals, vector<int> discreteVals, 
            function<double(array<double, 3>, double, glm::dvec3, double, double, 
            double, double, vector<string>, vector<double>)> scoreFunc, array<double, 3>& bestConfig, 
            double volMeshRes, double surfMeshRes, int procRank, int nCPUsPerRank);


        //void plot(int SCREEN_WIDTH, int SCREEN_HEIGHT, vector<string> paramNames, vector<double> paramVals, vector<int> discreteVals, double volMeshRes);

    private:

        //Finds the variables associated with the volumetric mesh, including the bounding box of each part
        void getVolVals(const profile& partProfile, const extrusionData& extrusion, double density,
            double& mass, glm::dvec3& COM, glm::dmat3& MOI, glm::dmat2x3& boundingBox) const;
        
        void getExtrusionData(vector<profile>& profiles, vector<extrusionData>& extrusions, 
            vector<string> paramNames, vector<double> paramVals, vector<int> discreteVals, double volMeshRes) const;
        //Gets relational matrix used in translation of MOI. Returns relational matrix
        glm::dmat3 constructRelationMatrix(glm::dvec3 r) const;

        //Gets COMs and MOIs at every position
        //Parameter positionVariables follows same format as in getAeroVals
        //For motor parameters, inner vector is list of motors, outer vector is list of positions
        void getPhysVals(vector<vector<double>> positionVariables,
            double staticMass, glm::dvec3 staticCOM, glm::dmat3 staticMOI, vector<double> controlMasses, 
            vector<glm::dvec3> controlPivots, vector<glm::dvec3> controlAxes, vector<glm::dvec3> controlCOMs,
            vector<glm::dmat3> controlMOIs,
            vector<glm::dvec3>& COMs, vector<glm::dmat3>& MOIs);

        //Gets the aerodynamic forces (net force, torque) on the aircraft for a range of configurations of the aircraft
        //Forces are normalised for velocity squared
        //The first two columns in positonVariables are values of pitch and yaw, the rest are control surface positions
        pair<vector<glm::dvec3>, vector<glm::dvec3>> getAeroVals(vector<vector<double>> positionVariables, 
            const vector<double>& staticSDF, glm::ivec3 SDFSize, const vector<glm::dvec3>& XYZ,
            const vector<profile>& profiles, vector<extrusionData> extrusions, vector<int> controlSurfaces,
            vector<glm::dvec3> controlAxes, vector<glm::dvec3> controlPivots, int horizontalStabiliser,
            int elevatorPart, const vector<double>& horizontalStabiliserSDF, 
            vector<glm::dvec3> totalCOMs, const vector<glm::dmat2x3>& boundingBoxes, 
            glm::dmat2x3 totalBoundingBox, vector<double>& tEfficiencyFactors, 
            vector<glm::dvec3>& tailForce, vector<glm::dvec3>& tailTorques, double surfMeshRes,
            int procRank, int nCPUsPerRank);

        //Finds the velocity at a given configuration
        double calculateVelocity(vector<extrusionData> extrusions, vector<int> motorParts, 
            double throttle, double dragCoeff, vector<glm::dvec3> motorThrustDirs, 
            double alpha, double yaw) const;

        //Names of all of the parameters, in order for searching
        vector<string> parameterNames;
        vector<string> fullParamNames;

        //Function used to calculate derived values from parameters
        //The four doubles are reference values: wing root chord, wing length,
        //wing scale, horizontal stabiliser area
        function<void(vector<string>&, vector<double>&, const vector<dataTable>&, 
            vector<int>, double&, double&, double&, double&)> derivedParamsFunc;
        //Functions used to produce profiles based on parameters
        vector<function<profile(vector<string>, vector<double>, double)>> profileFunctions;
        //Functions used to produce extrusion information based on parameters
        vector<function<extrusionData(vector<string>, vector<double>, double)>> extrusionFunctions;

        vector<string> partNames;
        //vector<int> partParents;
        vector<vector<int>> parentIndices;

        vector<dataTable> discreteTables;
        //Stores the index of the profile used for each part
        vector<int> partProfiles;
        vector<double> partDensities;

        //vector<int> controlSurfaces;

        //Describes the surface height deviation
        double roughnessHeight;

};
