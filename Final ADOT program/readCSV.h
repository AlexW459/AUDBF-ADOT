#pragma once

#include <string>
#include <vector>
#include <utility> 

using namespace std;

//Members: vector<string> rowNames, vector<pair<string, vector<float>>> columns
struct dataTable {
    vector<string> rowNames;
    vector<pair<string, vector<float>>> columns;

};

dataTable readCSV(string fileName);
