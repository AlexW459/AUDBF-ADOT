#pragma once

#include <vector>
#include <string>
#include <functional>
#include <stdexcept>
#include "dataTable.h"
#include "derParamFunc.h"
#include "scoreFunc.h"
#include "profileFunc.h"
#include "extrusionFunc.h"
#include "../../Mesh_Generation/profile/profile.h"
#include "../../Mesh_Generation/extrusion/extrusionData.h"


struct testModel{

    inline testModel(){};

    inline testModel(std::vector<std::string> _paramNames, std::vector<glm::dvec2> _paramRanges,
        std::vector<dataTable> _discreteTables, derParamFuncType _derivedParamsFunc, 
        std::vector<profileFuncType> _profileFunctions,
        std::vector<std::vector<double>> _positionVariables, 
        scoreFuncType _scoreFunc) : paramNames(_paramNames),
        paramRanges(_paramRanges), discreteTables(_discreteTables), derivedParamsFunc(_derivedParamsFunc),
        profileFuncs(_profileFunctions), scoreFunc(_scoreFunc), simPositionVariables(_positionVariables) {};

    inline void addPart(std::string partName, double density,
        extrusionFuncType extrusionFunction, int profileIndex){
            partNames.push_back(partName);
            partDensities.push_back(density);
            extrusionFuncs.push_back(extrusionFunction);
            parentIndices.push_back({});
            partProfiles.push_back(profileIndex);
        };

    inline void addPart(std::string partName, std::string parentPart, double density,
        extrusionFuncType extrusionFunction, int profileIndex){
            partNames.push_back(partName);
            partDensities.push_back(density);
            extrusionFuncs.push_back(extrusionFunction);
            partProfiles.push_back(profileIndex);

            // Gets parent indices
            int parentIndex = (int)(std::find(partNames.begin(), partNames.end(), parentPart) - partNames.begin());
            //Checks that specified part exists
            if(parentIndex == (int)partNames.size())
                throw std::runtime_error("Could not find parent \"" + parentPart + "\" when adding part \"" + partName + "\"");
            
            std::vector<int> partParentIndices = {parentIndex};
            partParentIndices.insert(partParentIndices.begin() + 1, parentIndices[parentIndex].begin(), 
                parentIndices[parentIndex].end());
            parentIndices.push_back(partParentIndices);
        };

    /*#ifdef USE_SDL
        void plot(int SCREEN_WIDTH, int SCREEN_HEIGHT, std::vector<extrusion> extrusions);
    #endif*/

    // Parameter information
    std::vector<std::string> paramNames;
    std::vector<glm::dvec2> paramRanges;
    std::vector<dataTable> discreteTables;

    // Functions
    derParamFuncType derivedParamsFunc;
    std::vector<profileFuncType> profileFuncs;
    std::vector<extrusionFuncType> extrusionFuncs;
    scoreFuncType scoreFunc;

    // List of positions to conduct aerodynamic simulations at. First three values are 
    // air velocity in x, y and z directions. Each following value is the angle of a 
    // control surface, in the order they are added
    std::vector<std::vector<double>> simPositionVariables;

    // Stores part info
    std::vector<std::string> partNames;
    std::vector<std::vector<int>> parentIndices;

    //Stores the index of the profile used for each part
    std::vector<int> partProfiles;

    //Stores the density of each part
    std::vector<double> partDensities;

};