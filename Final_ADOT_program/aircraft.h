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

#include "Mesh_Generation/meshWindow.h"
#include "Mesh_Generation/profile.h"
#include "Mesh_Generation/surfaceMeshGen.h"
#include "Mesh_Generation/extrusionGen.h"



using namespace std;



class aircraft{
    public:
        aircraft(vector<string> _paramNames, function<void(vector<string>&, vector<double>&)>
            _derivedParamsFunc, 
            vector<function<profile(vector<string>, vector<double>, double)>> _profileFunctions);

        void addPart(string partName, double density,
            function<extrusionData(vector<string>, vector<double>, double)> extrusionFunction, int profileIndex);
        void addPart(string partName, string parentPart, double density,
            function<extrusionData(vector<string>, vector<double>, double)> extrusionFunction, int profileIndex);

        int findPart(string partName);

        void calculateVals(vector<double> paramValues, double volMeshRes, double surfMeshRes,
            double &mass, glm::dvec3 &COM, glm::dmat3 &MOI);

        void plot(int SCREEN_WIDTH, int SCREEN_HEIGHT, vector<double> paramVals, double volMeshRes);

    private:

        vector<int> findParents(int partIndex) const;

        //Finds the variables associated with the volumetric mesh, including the bounding box of each part
        void findVolVals(const profile& partProfile, const extrusionData& extrusion, double& volume, glm::dvec3& COM, 
            glm::dmat3& MOI, glm::dmat2x3& boundingBox) const;
        
        void getExtrusionData(vector<profile>& profiles, vector<extrusionData>& extrusions, vector<double> paramValues, double volMeshRes) const;
        //Gets relational matrix used in translation of MOI. Returns relational matrix
        glm::dmat3 constructRelationMatrix(glm::dvec3 r) const;

        void getAeroVals();

        void getFlightPerformance();

        //Names of all of the parameters, in order for searching
        vector<string> parameterNames;
        //Function used to calculate derived values from parameters
        function<void(vector<string>&, vector<double>&)> derivedParamsFunc;
        //Functions used to produce profiles based on parameters
        vector<function<profile(vector<string>, vector<double>, double)>> profileFunctions;
        //Functions used to produce extrusion information based on parameters
        vector<function<extrusionData(vector<string>, vector<double>, double)>> extrusionFunctions;

        vector<string> partNames;
        vector<int> partParents;
        //Stores the index of the profile used for each part
        vector<int> partProfiles;
        vector<double> partDensities;

};