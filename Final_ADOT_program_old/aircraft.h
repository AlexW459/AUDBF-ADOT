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
#include "Mesh_Generation/MC.h"
#ifdef USE_SDL
#include "Mesh_Generation/meshWindow.h"
#endif

#include "getForces.h"


//Members: vector<string> rowNames, vector<pair<string, vector<float>>> columns
struct dataTable {
    vector<string> colNames;
    vector<pair<string, vector<double>>> rows;

};

using namespace std;

typedef function<double(vector<string> fullParamNames, vector<double> fullParamVals, 
    vector<dataTable> discreteTables, vector<int> discreteVals, vector<vector<double>> positionVariables,
    double mass, vector<glm::dvec3> COMs, vector<glm::dmat3> MOIs, vector<glm::dvec3> totalForces, 
    vector<glm::dvec3> totalTorques, vector<vector<glm::dvec3>> regionForces,
    vector<vector<glm::dvec3>> regionTorques, vector<vector<double>> regionVelMags,
    vector<vector<glm::dvec3>> POIs, vector<glm::dvec3> partDirections)> scoreFuncType;

typedef function<void(vector<string>& paramNames, vector<double>& paramVals, 
    const vector<dataTable>& discreteTables, vector<int> discreteVals, 
    vector<glm::dmat2x3>& velRegions)> derParamFuncType;

class aircraft{
    public:
        aircraft(vector<string> _paramNames, vector<glm::dvec2> _paramRanges,
            vector<dataTable> _discreteTables, derParamFuncType _derivedParamsFunc, 
            vector<function<profile(vector<string>, vector<double>, double)>> _profileFunctions,
            vector<vector<double>> _positionVariables, scoreFuncType _scoreFunc, double _roughnessHeight);

        void addPart(string partName, double density,
            function<extrusionData(vector<string>, vector<double>, double)> extrusionFunction, int profileIndex);
        void addPart(string partName, string parentPart, double density,
            function<extrusionData(vector<string>, vector<double>, double)> extrusionFunction, int profileIndex);

        int findPart(string partName);

        //ScoreFunc parameters are: configuration variables (AOA, elevator, throttle), 
        //aerodynamic forces, velocity, oscillation frequency, damping coefficient, 
        //dMdalpha, mass, paramNames, paramVals
        double calculateScore(vector<double> paramVals, vector<int> discreteVals, 
            array<double, 3>& bestConfig, vector<double> simParams, bool writeObjs, int procRank, 
            int nSimNodes, int meshParallelOpt, int simParallelOpt, int nSimTasksPerNode);

        #ifdef USE_SDL
            void plot(int SCREEN_WIDTH, int SCREEN_HEIGHT, vector<string> paramNames, vector<double> paramVals, vector<int> discreteVals, double volMeshRes);
        #endif


        // Names of all of the parameters, in order for searching
        vector<string> paramNames;

        // Ranges of parameter values
        vector<glm::dvec2> paramRanges;
        vector<dataTable> discreteTables;

    private:

        // Finds the variables associated with the volumetric mesh, including the bounding box of each part
        void getVolVals(const profile& partProfile, const extrusionData& extrusion, double density,
            double& mass, glm::dvec3& COM, glm::dmat3& MOI, glm::dmat2x3& boundingBox) const;
        
        void getExtrusionData(vector<profile>& profiles, vector<extrusionData>& extrusions, 
            vector<string> paramNames, vector<double> paramVals, vector<int> discreteVals, double volMeshRes) const;
        //Gets relational matrix used in translation of MOI. Returns relational matrix
        glm::dmat3 constructRelationMatrix(glm::dvec3 r) const;

        //Gets COMs and MOIs at every position
        //Parameter positionVariables follows same format as in getAeroVals
        void getPhysVals(vector<vector<double>> positionVariables,
            double staticMass, glm::dvec3 staticCOM, glm::dmat3 staticMOI, vector<double> controlMasses, 
            vector<glm::dvec3> controlPivots, vector<glm::dvec3> controlAxes, vector<glm::dvec3> controlCOMs,
            vector<glm::dmat3> controlMOIs, vector<glm::dvec3>& COMs, vector<glm::dmat3>& MOIs);

        //Gets the aerodynamic forces (net force, torque) on the aircraft for a range of configurations of the aircraft
        //Forces are normalised for velocity squared
        //The first three columns of positionVariables is the vector of airflow, the next three are the gravity unit vector,
        //any additional columns are angles ofcontrol surfaces
        pair<vector<glm::dvec3>, vector<glm::dvec3>> getAeroVals(vector<vector<double>> positionVariables, 
            const vector<double>& staticSDF, glm::ivec3 SDFSize, const vector<glm::dvec3>& XYZ,
            const vector<profile>& profiles, vector<extrusionData> extrusions, vector<int> controlSurfaces,
            vector<glm::dvec3> controlAxes, vector<glm::dvec3> controlPivots, vector<int> forceRegionParts,
            vector<glm::dmat2x3> velRegions, vector<glm::dvec3> totalCOMs, const vector<glm::dmat2x3>& boundingBoxes, 
            glm::dmat2x3 totalBoundingBox, vector<vector<glm::dvec3>>& regionForces, vector<vector<glm::dvec3>>& regionTorques, 
            vector<vector<double>>& regionAvgVels, vector<double> simParams, bool writeObjs, int procRank, 
            int meshParallelOpt, int simParallelOpt, int nSimNodes, int nSimTasksPerNode);

        



        //Function used to calculate derived values from parameters
        //Arguments are: param names, param values, discrete data tables,
        // discrete values, and a vector of bounds of velocity regions
        derParamFuncType derivedParamsFunc;
        //Functions used to produce profiles based on parameters
        vector<function<profile(vector<string>, vector<double>, double)>> profileFunctions;
        //Functions used to produce extrusion information based on parameters
        vector<function<extrusionData(vector<string>, vector<double>, double)>> extrusionFunctions;

        //User provided function used to evaluate score based on simulation results
        scoreFuncType scoreFunc;

        // List of positions to conduct aerodynamic simulations at. First three values are 
        // air velocity in x, y and z directions. Each following value is the angle of a 
        // control surface, in the order they are added
        vector<vector<double>> simPositionVariables;

        // Stores part info
        vector<string> partNames;
        vector<vector<int>> parentIndices;

        //Stores the index of the profile used for each part
        vector<int> partProfiles;
        vector<double> partDensities;

        //vector<int> controlSurfaces;

        //Describes the surface height deviation
        double roughnessHeight;

};
