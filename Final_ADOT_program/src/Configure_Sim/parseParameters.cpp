#include "parseParameters.h"

using namespace std;

void parseParameters(int argc, char *argv[], string parameterFileName, int& meshParallelOpt,
    int& simParallelOpt, int& nSimNodes, int& nSimTasksPerNode, map<string, double>& simParameters,
    bool& writeObjs, optim::algo_settings_t& optimSettings, void*& constructModelFunc, void*& objFileHandle)
{

    meshParallelOpt = NOT_PARALLEL;
    simParallelOpt = NOT_PARALLEL;
    writeObjs = false;
    nSimNodes = 1;
    nSimTasksPerNode = 1;


    //cout << argc << endl;

    // First argument is always the object file
    if (argc == 1){
        throw runtime_error("Error: please specify object file");
    }
    string objFileName = argv[1];
    //Opens object file
    constructModelFunc = loadObjectFile(objFileName, objFileHandle);
    
    // Loops through arguments
    int argNum = 2;
    while (argNum < argc){
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
        // Parallel simulation option
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
        // Just runs meshing and writes objs to file without running simulation
        }else if(string(argv[argNum]) == "-writeObjs"){
            argNum++;
            writeObjs = true;
        }else{
            throw runtime_error("Unrecognised command line option \"" + 
                    string(argv[argNum]) + "\"");
            argNum++;
        }
    }

    //Reads parameter file
    ifstream parameterFile;
    string newLine;
    parameterFile.open(parameterFileName);

    if(!parameterFile) throw runtime_error("Could not open file \"" + parameterFileName + "\" in file \"parseParameters.cpp\"");

    regex lineMatch("(^[a-zA-Z_\\d]+)[ \\t]+([+-]?\\d+(?:.\\d+)?(?:[eE][+-]?\\d+(?:.\\d+)?)?)");
    regex whitespaceMatch("^ *(?:\\/\\/)?");
    smatch lineInfo;
    smatch lineWhitespace;

    int nParams = 18;
    array<string, 18> paramNames = {"PROFILE_RESOLUTION", "SDF_RESOLUTION",
        "SDF_BAND_WIDTH", "BOUND_MULTIPLE", "CELL_GRADIENT", "IBM_SIMULATION", 
        "SIMULATION_LENGTH", "SIMULATION_LENGTH_INITIAL", "SIMULATION_DELTA_T",
        "SIMULATION_WRITE_INTERVAL", "RHO", "GEN_OPTIM", "OPTIM_PRINT_OPT",
        "CONV_FAILURE_SWITCH", "MAX_OPTIM_ITER", "GRAD_ERR_TOL", "REL_SOL_CHANGE_TOL",
        "REL_OBJFN_CHANGE_TOL"};
    for(int i = 0; i < nParams; i++){simParameters.insert({paramNames[i], NAN});}

    int nGDparams = 16;
    array<string, 16> gdParamNames = {"PAR_STEP_SIZE", "STEP_DECAY", "STEP_DECAY_PERIODS", "STEP_DECAY_VAL", "PAR_MOMENTUM", 
        "PAR_ADA_NORM_TERM", "PAR_ADA_RHO", "ADA_MAX", "PAR_ADAM_BETA_1", "PAR_ADAM_BETA_2", "CLIP_GRAD",
        "CLIP_MAX_NORM", "CLIP_MIN_NORM", "CLIP_NORM_TYPE", "CLIP_NORM_BOUND", "FINITE_DIFFERENCE_H"};
    map<string, double> gdParameters;
    optim::gd_settings_t gdSettings;
    for(int i = 0; i < nGDparams; i++){gdParameters.insert({gdParamNames[i], NAN});}

    int nGenParams = 7;
    array<string, 7> genParamNames = {"N_POP", "N_POP_BEST", "N_GEN", "MUTATION_METHOD", "CHECK_FREQ", 
        "PAR_F", "PAR_CR"};
    map<string, double> genParameters;
    optim::de_settings_t genSettings;
    for(int i = 0; i < nGenParams; i++){genParameters.insert({genParamNames[i], NAN});}

    int lineNum = 0;
    while(getline(parameterFile, newLine)){
        lineNum++;
        bool matched = regex_search(newLine, lineInfo, lineMatch);
        bool whitespaceMatched = regex_search(newLine, lineWhitespace, whitespaceMatch);
        if(matched){
            if(simParameters.find(lineInfo.str(1)) != simParameters.end()){
                simParameters[lineInfo.str(1)] = stod(lineInfo.str(2));
            }else if(gdParameters.find(lineInfo.str(1)) != gdParameters.end()){
                gdParameters[lineInfo.str(1)] = stod(lineInfo.str(2));
            }else if(genParameters.find(lineInfo.str(1)) != genParameters.end()){
                genParameters[lineInfo.str(1)] = stod(lineInfo.str(2));
            }else{
                throw runtime_error("Unrecognised parameter \"" + lineInfo.str(1) + "\" on line " + to_string(lineNum) + " of parameter file");
            }
        }else if (!whitespaceMatched){
            throw runtime_error("Unrecognised expression \"" + newLine + "\" on line " + to_string(lineNum) + " of parameter file");
        }
    }

    // Checks that all parameters are present
    for(map<string, double>::iterator i = simParameters.begin(); i != simParameters.end(); i++){
        if(isnan(i->second)){
            throw runtime_error("Parameter \"" + i->first + "\" missing from parameter file");
        }
    }

    if(!simParameters["GEN_OPTIM"]){
        // Checks that all gd parameters are present
        for(map<string, double>::iterator i = gdParameters.begin(); i != gdParameters.end(); i++){
            if(isnan(i->second)){
                throw runtime_error("Parameter \"" + i->first + "\" required for gradient descent missing from parameter file");
            }
        }

        optimSettings.gd_settings.method = 0;
        optimSettings.gd_settings.step_decay = gdParameters["STEP_DECAY"];
        optimSettings.gd_settings.step_decay_periods = gdParameters["STEP_DECAY_PERIODS"];
        optimSettings.gd_settings.step_decay_val = gdParameters["STEP_DECAY_VAL"];
        optimSettings.gd_settings.par_momentum = gdParameters["PAR_MOMENTUM"];
        optimSettings.gd_settings.par_ada_norm_term = gdParameters["PAR_ADA_NORM_TERM"];
        optimSettings.gd_settings.par_ada_rho= gdParameters["PAR_MOMENTUM"];
        optimSettings.gd_settings.ada_max = gdParameters["PAR_ADA_NORM_TERM"];
        optimSettings.gd_settings.par_adam_beta_1 = gdParameters["PAR_ADA_RHO"];
        optimSettings.gd_settings.par_adam_beta_2 = gdParameters["ADA_MAX"];
        optimSettings.gd_settings.clip_grad = gdParameters["PAR_ADAM_NORM_BETA_1"];
        optimSettings.gd_settings.clip_max_norm = gdParameters["PAR_ADAM_NORM_BETA_2"];
        optimSettings.gd_settings.clip_min_norm = gdParameters["CLIP_GRAD"];
        optimSettings.gd_settings.clip_norm_type = gdParameters["CLIP_MAX_NORM"];
        optimSettings.gd_settings.clip_norm_bound = gdParameters["CLIP_MIN_NORM"];
        optimSettings.gd_settings.clip_norm_bound = gdParameters["CLIP_NORM_TYPE"];
        optimSettings.gd_settings.clip_norm_bound = gdParameters["CLIP_NORM_BOUND"];

        simParameters.insert({"FINITE_DIFFERENCE_H", gdParameters["FINITE_DIFFERENCE_H"]});
    
    }else{
        // Checks that all gen parameters are present
        for(map<string, double>::iterator i = genParameters.begin(); i != genParameters.end(); i++){
            if(isnan(i->second)){
                throw runtime_error("Parameter \"" + i->first + "\" required for differential evolution missing from parameter file");
            }
        }
        
        optimSettings.de_settings.n_pop = genParameters["N_POP"];
        optimSettings.de_settings.n_pop_best = genParameters["N_POP_BEST"];
        optimSettings.de_settings.n_gen = genParameters["N_GEN"];
        optimSettings.de_settings.mutation_method = genParameters["MUTATION_METHOD"];
        optimSettings.de_settings.check_freq = genParameters["CHECK_FREQ"];
        optimSettings.de_settings.par_F = genParameters["PAR_F"];
        optimSettings.de_settings.par_CR = genParameters["PAR_CR"];
        
    }

    optimSettings.print_level = simParameters["OPTIM_PRINT_OPT"];
    optimSettings.conv_failure_switch = simParameters["CONV_FAILURE_SWITCH"];
    optimSettings.iter_max = simParameters["MAX_OPTIM_ITER"];
    optimSettings.grad_err_tol = simParameters["GRAD_ERR_TOL"];
    optimSettings.rel_sol_change_tol = simParameters["REL_SOL_CHANGE_TOL"];

    parameterFile.close();
}
