#include "main.h"


using namespace std;

int main(int argc, char *argv[]) {
    
    MPI_Init(NULL, NULL);

    int procRank;
    int nProcs;

    MPI_Comm_rank(MPI_COMM_WORLD, &procRank);
    MPI_Comm_size(MPI_COMM_WORLD, &nProcs);

    

    void* objFileHandle;
    {

    // Gets arguments and simulation parameters
    string parameterFileName = "simulation_parameters";
    map<string, double> simParameters;
    int meshParallelOpt, simParallelOpt, nSimNodes, nSimTasksPerNode;
    bool writeObjs;


    void* constructModelPointer;
    optim::algo_settings_t optimSettings;
    parseParameters(argc, argv, parameterFileName, meshParallelOpt,
        simParallelOpt, nSimNodes, nSimTasksPerNode, simParameters, writeObjs,
        optimSettings, constructModelPointer, objFileHandle);


    // Initialise rand()
    srand (time(0));
    default_random_engine rndNumGenerator;
    rndNumGenerator.seed(time(0));

    // Initialise SDL
    #ifdef USE_SDL
        SDL_Init(SDL_INIT_EVERYTHING);
    #endif
    
    
    // Initialises model from function
    testModel (*constructModelFunc)() =  reinterpret_cast<testModel(*)()>(constructModelPointer);
    testModel model = constructModelFunc();

    
    
    int nParams = model.paramRanges.size();
    vector<double> paramVals(nParams);

    optim::ColVec_t lowerParamBounds(nParams);
    optim::ColVec_t upperParamBounds(nParams);

    for(int i = 0; i < nParams; i++){
        lowerParamBounds[i] = model.paramRanges[i][0];
        upperParamBounds[i] = model.paramRanges[i][1];
    }

    ScoreEvaluator::setSimParams(model, simParameters, upperParamBounds, lowerParamBounds, writeObjs, nSimNodes, simParallelOpt, 
        nSimTasksPerNode, procRank, nProcs);
    
    if(procRank == 0){


        string deleteDirBash = "rm -r -f Aerodynamics_Simulation_BFM_* Aerodynamics_Simulation_IBM_*";
        int failure = system(deleteDirBash.c_str());
        if(failure) throw runtime_error("Failed to clean old case directories before simulations");

        cout << "Cleaned old directories" << endl;

        //Creates a unique case directory for each position
        string originalCaseDir;
        string caseDir;
        if(simParameters["IBM_SIMULATION"]) {
            originalCaseDir = string(projectRoot) + "/Aerodynamics_Simulation_IBM";
            caseDir = "Aerodynamics_Simulation_IBM_";
        }
        else {
            originalCaseDir = string(projectRoot) + "/Aerodynamics_Simulation_BFM";
            caseDir = "Aerodynamics_Simulation_BFM_";
        }

        for(int i = 0; i < (int)model.simPositionVariables.size(); i++){
            string copyBash = "cp -r " + originalCaseDir + " " + caseDir + to_string(i);
            int failure = system(copyBash.c_str());
            if(failure) throw runtime_error("Failed to copy simulation directory for case " + to_string(i));
            
        }

        cout << "Entering optimisation loop" << endl;

        auto enterLoopTime = chrono::high_resolution_clock::now();

        optim::ColVec_t initialParams(nParams);
        
        optimSettings.vals_bound = true;
        optimSettings.lower_bounds = lowerParamBounds;
        optimSettings.upper_bounds = upperParamBounds;


        for(int i = 0; i < nParams; i++){
            initialParams[i] = model.paramRanges[i][0] + 1e-1;//model.paramRanges[i][0] + (rand() / (double)RAND_MAX)  * (model.paramRanges[i][1] - model.paramRanges[i][0]);
        }

        //Optimisation loop
        bool success;
        if(!simParameters["GEN_OPTIM"]){
            success = optim::gd(initialParams, ScoreEvaluator::calculateScore, nullptr, optimSettings);
        }else{
            optimSettings.de_settings.initial_lb = lowerParamBounds;
            optimSettings.de_settings.initial_ub = upperParamBounds;
            optimSettings.de_settings.return_population_mat = false;
            success = optim::de(initialParams, ScoreEvaluator::calculateScore, nullptr, optimSettings);
        }

        auto duration = chrono::duration_cast<chrono::seconds>(
            chrono::high_resolution_clock::now() - enterLoopTime);

        if(!success){
            cout << "Optimisation failed after " << duration.count() << " seconds" << endl;
        }else{
            cout << "Optimisation finished after " << duration.count() << " seconds" << endl;
            cout << "Final Values:" << endl;
            for(int i = 0; i < nParams; i++){
                cout << model.paramNames[i] << ": " << initialParams[i] << endl;
            }
        }

    }else{

        bool exitFlag = false;
        MPI_Recv(&exitFlag, 1, MPI_CXX_BOOL, 0, EXIT_PROGRAM, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        while (!exitFlag){
            optim::ColVec_t paramVec(nParams);

            cout << "Entering score func on rank " << procRank << endl;
            
            ScoreEvaluator::calculateScore(paramVec, nullptr, nullptr);

            MPI_Recv(&exitFlag, 1, MPI_CXX_BOOL, 0, EXIT_PROGRAM, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        }
    }

    }

    // Tells all other ranks to exit
    if(procRank == 0){
        bool trueVal = true;
        for(int t = 1; t < nProcs; t++){
            MPI_Send(&trueVal, 1, MPI_CXX_BOOL, t, EXIT_PROGRAM, MPI_COMM_WORLD);
        }
    }

    cout << "Exiting on rank " << procRank << endl;

    ScoreEvaluator::dealloc();

    // Closes the shared library
    dlclose(objFileHandle);

    //Quit SDL
    #ifdef USE_SDL
        SDL_Quit();
    #endif

    MPI_Finalize();

    return 0;
}
