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
        //Checks for whitespace at the end of the string (not allowed)
        if(isspace(val[val.length() - 1] )) val.erase(val.end() - 1);

        csvContents.colNames.push_back(val);
    }

    
    int rowNum = 0;

    //Fills up rows
    while(getline(csvFile, line)){
        //Gets first value in column, always a string
        stringstream row(line);
        getline(row, val, ',');
        vector<double> rowVals;

        csvContents.rows.push_back(make_pair(val, rowVals));
        //Fills up columns in row
        int col = 0;

        while(getline(row, val, ',')){
            try{
                csvContents.rows[rowNum].second.push_back( stod(val));
            } catch(int e){
                throw std::runtime_error("Invalid table data in row " + to_string(rowNum+1) + ", column " + 
                    to_string(col) + ". " + val + " is not a valid value.Must be a number");
            }
            col++;
        }

        rowNum++;
    }

    csvFile.close();

    return csvContents;
} 