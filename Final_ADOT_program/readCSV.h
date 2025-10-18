#pragma once

#include <string>
#include <vector>
#include <utility> 
#include <algorithm>
#include <iostream>

using namespace std;

//Members: vector<string> rowNames, vector<pair<string, vector<float>>> columns
struct dataTable {
    vector<string> colNames;
    vector<pair<string, vector<double>>> rows;

};

dataTable readCSV(string fileName);
