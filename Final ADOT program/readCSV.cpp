#include <fstream>
#include <stdexcept>
#include <sstream>

#include "readCSV.h"


//Reads data from a CSV into a dataTable struct
dataTable readCSV(string fileName){

    dataTable csvContents;

    ifstream csvFile(fileName);
    if(!csvFile.is_open()) 
    throw std::runtime_error("Could not open file \"" + fileName + "\" in file \"readCSV.cpp\"");

    //Gets first line of file
    string line, val;
    getline(csvFile, line);
    stringstream columnNames(line);

    //Moves past initial blank space in corner of file
    getline(columnNames, val, ',');
    //Fills up vector of column headings
    while(getline(columnNames, val, ',')){
        vector<float> valueVec;
        csvContents.columns.push_back(make_pair(val, valueVec));
    }

    
    //Fills up rows
    while(getline(csvFile, line)){
        //Gets first value in column, always a string
        stringstream row(line);
        getline(row, val, ',');

        csvContents.rowNames.push_back(val);
        //Fills up columns
        int col = 0;
        while(getline(row, val, ',')){
            csvContents.columns[col].second.push_back( stof(val));
            col++;
        }
    }

    return csvContents;
} 