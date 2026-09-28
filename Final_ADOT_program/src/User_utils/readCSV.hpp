#pragma once

#include <string>
#include <vector>
#include <utility> 
#include <algorithm>
#include <iostream>
#include <bits/stdc++.h>

#include "../Score_Evaluation/testModel/dataTable.h"

//Reads data from a CSV into a dataTable struct
inline dataTable readCSV(std::string fileName){

    dataTable csvContents;

    std::ifstream csvFile(fileName);
    if(!csvFile.is_open()) 
    throw std::runtime_error("Could not open file \"" + fileName + "\" in file \"readCSV.cpp\"");

    //Gets first line of file
    std::string line, val;
    getline(csvFile, line);
    std::stringstream columnNames(line);

    //Moves past initial blank space in corner of file
    getline(columnNames, val, ',');
    //Fills up vector of column headings
    while(getline(columnNames, val, ',')){
        //Checks for whitespace at the end of the string (not allowed)
        if(isspace(val[val.length() - 1] )) val.erase(val.end() - 1);

        csvContents.colNames.push_back(val);
    }

    
    int rowNum = 0;

    //Fills up rows
    while(getline(csvFile, line)){
        //Gets first value in column, always a string
        std::stringstream row(line);
        getline(row, val, ',');
        std::vector<double> rowVals;

        csvContents.rows.push_back(make_pair(val, rowVals));
        //Fills up columns in row
        int col = 0;


        while(getline(row, val, ',')){
            //Checks if string is whitespace, because error detection doesn't always work properly in this case
            if(isspace(val[0])){
                throw std::runtime_error("Entry in row " + std::to_string(rowNum+1) + 
                ", column " + std::to_string(col+1) + " in table \'" + fileName + "\' is empty");
            }

            try{
                csvContents.rows[rowNum].second.push_back( stod(val));
            } catch(std::invalid_argument& e){
                throw std::runtime_error("Invalid data in row " + std::to_string(rowNum+1) + ", column " + 
                    std::to_string(col+1) + " in table \'" + fileName + "\'. " + val + " is not a valid value. Must be a number");
            }
            col++;
        }

        rowNum++;
    }

    csvFile.close();


    return csvContents;
} 
