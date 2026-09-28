#pragma once

#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <algorithm>
#include <stdexcept>

inline double getParam(std::string param, const std::vector<double>& paramVals, 
    const std::vector<std::string>& paramNames){

    if(paramNames.size() != paramVals.size()){
        throw std::runtime_error("Difference in length between parameter values list and parameter names list");
    }

    int numParams = paramVals.size();
    int index = numParams;
    for(int i = 0; i < numParams; i++){
        if(paramNames[i] == param){
            index = i;
            break;
        }
    }

    if(index == numParams){
        throw std::runtime_error("Could not find variable \"" + param + "\" in parameter list");
        return 0.0;
    }else{
        return paramVals[index];
    }
}