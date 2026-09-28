#pragma once

#include <vector>
#include <string>

//Members: vector<string> rowNames, vector<pair<string, vector<float>>> columns
struct dataTable {
    std::vector<std::string> colNames;
    std::vector<std::pair<std::string, std::vector<double>>> rows;
};