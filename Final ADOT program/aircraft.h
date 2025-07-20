#pragma once

#include <vector>
#include <string>
#include <utility>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <stdexcept>
#include <iostream>

#include "Mesh_Generation/meshWindow.h"
#include "Mesh_Generation/profile.h"

using namespace std;

struct extrusionData{
    vector<float> zSampleVals;
    vector<float> yPosVals;
    vector<float> scaleVals;
    float sweep;

    glm::quat rotation;
    glm::vec3 translation;

    extrusionData(){};

    extrusionData(vector<float> _zSampleVals, vector<float> _yPosVals,
        vector<float> _scaleVals, float _sweep, glm::quat _rotation, 
        glm::vec3 _translation):zSampleVals(_zSampleVals), yPosVals(_yPosVals), 
        scaleVals(_scaleVals), sweep(_sweep), rotation(_rotation), translation(_translation){};

};



class aircraft{
    public:
        aircraft(vector<string> _paramNames, function<void(vector<string>&, vector<double>&)>
            _derivedParamsFunc, 
            vector<function<profile*(vector<string>, vector<double>, double)>> _profileFunctions);

        void addPart(string partName, float density,
            function<extrusionData(vector<string>, vector<double>, double)> extrusionFunction, int profileIndex);
        void addPart(string partName, string parentPart, float density,
            function<extrusionData(vector<string>, vector<double>, double)> extrusionFunction, int profileIndex);

        int findPart(string partName);
        vector<int> findParents(int partIndex);

        void calculateVals(vector<double> paramValues, double volMeshRes, double surfMeshRes,
            float &mass, glm::vec3 &COM, glm::mat3 &MOI);

        void plot(int SCREEN_WIDTH, int SCREEN_HEIGHT, vector<double> paramVals, double volMeshRes);

    private:
        //Finds the variables associated with the volumetric mesh
        void findVolVals(const profile& partProfile, const extrusionData& extrusion, float& mass, glm::vec3& COM, glm::mat3& MOI) const;
        
        void getExtrusionData(profile* profiles, extrusionData* extrusions, vector<double> paramValues, double volMeshRes) const;
        int generateExtrusion(const profile& partProfile, const extrusionData& extrusion, char**& adjMatrix, vector<glm::vec3>& points) const;
        glm::mat3 constructRelationMatrix(glm::vec3 r) const;

        //Names of all of the parameters, in order for searching
        vector<string> parameterNames;
        //Function used to calculate derived values from parameters
        function<void(vector<string>&, vector<double>&)> derivedParamsFunc;
        //Functions used to produce profiles based on parameters
        vector<function<profile*(vector<string>, vector<double>, double)>> profileFunctions;
        //Functions used to produce extrusion information based on parameters
        vector<function<extrusionData(vector<string>, vector<double>, double)>> extrusionFunctions;

        vector<string> partNames;
        vector<int> partParents;
        //Stores the index of the profile used for each part
        vector<int> partProfiles;
        vector<float> partDensities;
  
};