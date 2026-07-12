#include "parseParameters.h"

void parseParameters(int argc, char *argv[], string parameterFileName, int& meshParallelOpt,
    int& simParallelOpt, int& nSimNodes, int& nSimTasksPerNode, vector<double>& simParameters,
    bool& writeObjs)
{

    meshParallelOpt = NOT_PARALLEL;
    simParallelOpt = NOT_PARALLEL;
    writeObjs = false;
    nSimNodes = 1;
    nSimTasksPerNode = 1;
    
    // Loops through arguments
    int argNum = 1;
    while (argNum <= argc){
        // Checks for parallel mesh option
        if(string(argv[argNum]) == "-meshParallel"){
            argNum++;
            if (argc < argNum) throw runtime_error("Insufficient arguments following -meshParallel");
            if (string(argv[argNum]) == "-slurm"){
                meshParallelOpt = PARALLEL_SLURM;
                if (argc < argNum + 2) throw runtime_error("Insufficient arguments following -meshParallel -slurm");

                // Validates values as integers
                for (int i = 0; i < (int)strlen(argv[argNum]); i++) {
                    if(!isdigit(argv[argNum][i])) throw runtime_error("Unrecognised argument \"" + 
                    string(argv[argNum]) + "\" following -meshParallel -slurm");}
                for (int i = 0; i < (int)strlen(argv[argNum+1]); i++) {
                    if(!isdigit(argv[argNum+1][i])) throw runtime_error("Unrecognised argument \"" + 
                    string(argv[argNum+1]) + "\" following -meshParallel -slurm");}

                nSimNodes = atoi(argv[argNum]);
                nSimTasksPerNode = atoi(argv[argNum+1]);
                argNum +=2;
            }else{
                meshParallelOpt = PARALLEL;
                argNum++;
                for (int i = 0; i < (int)strlen(argv[argNum]); i++) {
                    if(!isdigit(argv[argNum][i])) throw runtime_error("Unrecognised command line option \"" + 
                    string(argv[argNum]) + "\" following -meshParallel");}
                
                nSimTasksPerNode = atoi(argv[argNum]);
            }
        }else if(string(argv[argNum]) == "-simParallel"){
            argNum++;
            if (argc < argNum) throw runtime_error("Insufficient arguments following -simParallel");
            if (string(argv[argNum]) == "-slurm"){
                simParallelOpt = PARALLEL_SLURM;
                if (argc < argNum + 2) throw runtime_error("Insufficient arguments following -simParallel -slurm");

                // Validates values as integers
                for (int i = 0; i < (int)strlen(argv[argNum]); i++) {
                    if(!isdigit(argv[argNum][i])) throw runtime_error("Unrecognised command line option \"" + 
                    string(argv[argNum]) + "\" following -simParallel -slurm");}
                for (int i = 0; i < (int)strlen(argv[argNum+1]); i++) {
                    if(!isdigit(argv[argNum+1][i])) throw runtime_error("Unrecognised command line option \"" + 
                    string(argv[argNum+1]) + "\" following -simParallel -slurm");}

                nSimNodes = atoi(argv[argNum]);
                nSimTasksPerNode = atoi(argv[argNum+1]);
                argNum +=2;
            }else{
                simParallelOpt = PARALLEL;
                argNum++;
                for (int i = 0; i < (int)strlen(argv[argNum]); i++) {
                    if(!isdigit(argv[argNum][i])) throw runtime_error("Unrecognised argument \"" + 
                    string(argv[argNum]) + "\" following -meshParallel");}
                
                nSimTasksPerNode = atoi(argv[argNum]);
            }

        }else if(string(argv[argNum]) == "-writeObjs"){
            argNum++;
            writeObjs = true;
        }else{
            argNum++;
            throw runtime_error("Unrecognised command line option \"" + 
                    string(argv[argNum]) + "\"");
        }
    }


    //Reads parameter file
    ifstream parameterFile;
    string newLine;
    parameterFile.open(parameterFileName);

    if(!parameterFile) throw runtime_error("Could not open file \"" + parameterFileName + "\" in file \"parseParameters.cpp\"");

    regex lineMatch("^[a-zA-Z_]+ +(\\d+(?:.\\d+))");
    regex whitespaceMatch("^ *(?:\\/\\/)?");
    smatch lineInfo;
    smatch lineWhitespace;
    int lineNum = 1;
    while(getline(parameterFile, newLine)){
        bool matched = regex_search(newLine, lineInfo, lineMatch);
        bool whitespaceMatched = regex_search(newLine, lineWhitespace, whitespaceMatch);
        if(matched){
            if(lineInfo.str(1) == "MUTATION_STD_DEV"){
                simParameters[0] = stod(lineInfo.str(2));
            }else if(lineInfo.str(1) == "PROFILE_RESOLUTION"){
                simParameters[0] = stod(lineInfo.str(2));
            }else if(lineInfo.str(1) == "SDF_RESOLUTION"){
                simParameters[0] = stod(lineInfo.str(2));
            }else if(lineInfo.str(1) == "SIMULATION_LENGTH"){
                simParameters[1] = stod(lineInfo.str(2));
            }else if(lineInfo.str(1) == "SIMULATION_DELTA_T"){
                simParameters[2] = stod(lineInfo.str(2));
            }else if(lineInfo.str(1) == "RHO"){
                simParameters[3] = stod(lineInfo.str(2));
            }else if(lineInfo.str(1) == "TURBULENCE_INTENSITY"){
                simParameters[4] = stod(lineInfo.str(2));
            }else if(lineInfo.str(1) == "TURBULENCE_LENGTH_SCALE"){
                simParameters[5] = stod(lineInfo.str(2));
            }else{
                throw runtime_error("Did not recognise parameter at line " + to_string(lineNum) + " of simulation_parameters");
            }
        }else if (!whitespaceMatched){
            throw runtime_error("Did not recognise line " + to_string(lineNum) + " of simulation_parameters");
        }

        lineNum++;
    }

    parameterFile.close();
}
