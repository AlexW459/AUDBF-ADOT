#include "main.h"


using namespace std;

int main(int argc, char *argv[]) {

    //Gets rank of node that program is being run on, the number of processes available 
    //to the program, and the total number of nodes
    int nodeRank = atoi(argv[1]);
    int nProcs = atoi(argv[2]);
    int nNodes = atoi(argv[3]);

    MPI_Init(&argc, &argv);

    //Initialise rand()
    srand (time(0));
    default_random_engine rndNumGenerator;
    rndNumGenerator.seed(time(0));
    //Initialise SDL
    //SDL_Init(SDL_INIT_EVERYTHING);

    string barrierFilename = "Comm_Files/Barrier";
    string paramFileName = "Comm_Files/paramVals";
    string discreteFileName = "Comm_Files/discreteVals";
    string scoresFileName = "Comm_Files/scores";

    //Initialises aircraft
    string planeModel = "MULEplaneModel";

    vector<function<profile(vector<string>, vector<double>, double)>> profileFunctions = 
        {fuselageProfile, wingProfile, motorPodProfile, empennageBoomProfile, 
            horizontalStabiliserProfile, elevatorProfile};
    
    //Gets maximum and minimun values of each parameter
    dataTable paramRanges = readCSV(planeModel + "/paramRanges.csv");
    dataTable motorTable = readCSV(planeModel + "/motorSelection.csv");
    dataTable batteryTable = readCSV(planeModel + "/batterySelection.csv");

    //Gets names of variables
    vector<string> paramNames(paramRanges.rows.size());
    for(int i = 0; i < (int)paramRanges.rows.size(); i++){
        paramNames[i] = paramRanges.rows[i].first;
    }
    vector<dataTable> discreteTables = {motorTable, batteryTable};

    
    aircraft MULEaircraft = aircraft(paramNames, discreteTables, calcDerivedParams, profileFunctions, 0.0005);

    MULEaircraft.addPart("fuselage", 1000, extrudeFuselage, 0);
    MULEaircraft.addPart("rightWing", "fuselage", 1000, extrudeRightWing, 1);
    MULEaircraft.addPart("leftWing", "fuselage", 1000, extrudeLeftWing, 1);
    MULEaircraft.addPart("motorPodRight", "rightWing", 1000, extrudeRightMotorPod, 2);
    MULEaircraft.addPart("motorPodLeft", "leftWing", 1000, extrudeLeftMotorPod, 2);
    MULEaircraft.addPart("empennageBoomRight", "motorPodRight", 1000, extrudeEmpennageBoom, 3);
    MULEaircraft.addPart("empennageBoomLeft", "motorPodLeft", 1000, extrudeEmpennageBoom, 3);
    MULEaircraft.addPart("horizontalStabiliser", "empennageBoomRight", 1000, extrudeHorizontalStabiliser, 4);
    MULEaircraft.addPart("elevator", "horizontalStabiliser", 1000, extrudeElevator, 5);


    vector<double> paramVals(paramNames.size());
    vector<int> discreteVals(discreteTables.size());
    
    int nParams = paramNames.size();
    int nDiscrete = discreteTables.size();


    //Finds random values of parameters within bounds
    for(int i = 0; i < nParams; i++){
        double min = paramRanges.rows[i].second[0];
        double max = paramRanges.rows[i].second[1];

        uniform_real_distribution paramDist(min, max);

        paramVals[i] = paramDist(rndNumGenerator);
    }

    //Finds random values of discrete parameters
    for(int i = 0; i < nDiscrete; i++){
        
        int nChoices = discreteTables[i].rows.size();

        uniform_int_distribution discreteDist(0, nChoices-1);

        discreteVals[i] = discreteDist(rndNumGenerator);
    }

    int nModels = nNodes;
    

    //Optimisation loop
    int numGenerations = 4;
    for(int generation = 0; generation < numGenerations; generation++){


        //Cleans up folders if needed
        if(nodeRank == 0) {
            int failure = system("rm -r -f Aerodynamics_Simulation_*");
            if(failure) throw runtime_error("Failed checking for and cleaning files");
        }

        MPI_Barrier(MPI_COMM_WORLD);

        //Rate aircraft performance
        array<double, 3> aircraftConfig;
        double score = MULEaircraft.calculateScore(paramVals, discreteVals, rateDesign, aircraftConfig, 
            75.0, 150.0, nodeRank, nProcs);

        cout << "score" << endl;

        //Wait until all nodes are finished
        MPI_Barrier(MPI_COMM_WORLD);


        vector<double> firstParentParamVals(nParams);
        vector<int>    firstParentDiscreteVals(nParams);
        vector<double> secondParentParamVals(nParams);
        vector<int>    secondParentDiscreteVals(nParams);


        if(nodeRank != 0){

            //Send info to controller node
            MPI_Send(&score, 1, MPI_DOUBLE, 0, 0, MPI_COMM_WORLD);
            MPI_Send(paramVals.data(), nParams, MPI_DOUBLE, 0, 1, MPI_COMM_WORLD);
            MPI_Send(discreteVals.data(), nDiscrete, MPI_INT, 0, 2, MPI_COMM_WORLD);

            //Get pair of new parameters
            MPI_Recv(firstParentParamVals.data(), nParams, MPI_DOUBLE, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
            MPI_Recv(secondParentParamVals.data(), nParams, MPI_DOUBLE, 0, 1, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
            MPI_Recv(firstParentDiscreteVals.data(), nDiscrete, MPI_INT, 0, 2, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
            MPI_Recv(secondParentDiscreteVals.data(), nDiscrete, MPI_INT, 0, 3, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

        }else{

            //Receives information
            vector<vector<double>> allParamVals(nModels, vector<double>(nParams));
            vector<vector<int>>    allDiscreteVals(nModels, vector<int>(nDiscrete));
            allParamVals[0] = paramVals;
            allDiscreteVals[0] = discreteVals;
            
            vector<double> scores(nModels);
            scores[0] = score;
            vector<double> interval(nModels);
            interval[0] = 0.0;

            for(int i = 1; i < nModels; i++){
                MPI_Recv(&(scores[i]), 1, MPI_DOUBLE, i, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                MPI_Recv(allParamVals[i].data(), nParams, MPI_DOUBLE, i, 1, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                MPI_Recv(allDiscreteVals[i].data(), nDiscrete, MPI_DOUBLE, i, 2, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                interval[i] = (double)i;
            }

            //Find pairs of parents using weighted random
            vector<pair<int, int>> parentPairs(nModels);

            //Weighted distribution
            piecewise_constant_distribution weightedDist(interval.begin(), interval.end(), scores.begin());

            //Pick first parent using weighted random, and then use weighted random on the remaining models to find the second
            for(int i = 0; i < nModels; i++){
                parentPairs[i].first = weightedDist(rndNumGenerator);

                //Generates second parent after first parent is excluded from pool
                vector<double> tempScores = scores;
                tempScores[parentPairs[i].first] = 0.0;
                piecewise_constant_distribution weightedDist2(interval.begin(), interval.end(), tempScores.begin());

                parentPairs[i].second = weightedDist2(rndNumGenerator);
            }

            firstParentParamVals = allParamVals[parentPairs[0].first];
            secondParentParamVals = allParamVals[parentPairs[0].second];
            firstParentDiscreteVals = allDiscreteVals[parentPairs[0].first];
            secondParentDiscreteVals = allDiscreteVals[parentPairs[0].second];

            //Send pairs of parent parameters to each node
            for(int i = 1; i < nModels; i++){
                MPI_Send(allParamVals[parentPairs[i].first].data(), nParams, MPI_DOUBLE, i, 0, MPI_COMM_WORLD);
                MPI_Send(allParamVals[parentPairs[i].second].data(), nParams, MPI_DOUBLE, i, 1, MPI_COMM_WORLD);
                MPI_Send(allDiscreteVals[parentPairs[i].first].data(), nDiscrete, MPI_INT, i, 2, MPI_COMM_WORLD);
                MPI_Send(allDiscreteVals[parentPairs[i].second].data(), nDiscrete, MPI_INT, i, 3, MPI_COMM_WORLD);
            }

        }

        MPI_Barrier(MPI_COMM_WORLD);

        //Do random crossover and mutations to find the parameters of the next model that will be tested on this node
        vector<double> crossParamVals(nParams);
        vector<int>    crossDiscreteVals(nDiscrete);

        
        
        //Pick each parameter of child entirely randomly
        uniform_int_distribution binaryDist(0, 1);
        
        //Randomly add or subtract a percentage of the maximum amount that can be added
        //or subtracted while staying in param range 
        uniform_real_distribution mutationDist(0.02, 0.2);
        for(int i = 0; i < nParams; i++){
            bool choice = binaryDist(rndNumGenerator);
            crossParamVals[i] = choice ? firstParentParamVals[i] : secondParentParamVals[i];
            crossDiscreteVals[i] = choice ? firstParentDiscreteVals[i] : secondParentDiscreteVals[i];
            

            bool addOrSubtract = binaryDist(rndNumGenerator);
            double min = addOrSubtract ? crossParamVals[i] : paramRanges.rows[i].second[0];
            double max = addOrSubtract ? paramRanges.rows[i].second[1] : crossParamVals[i];
            double mutateAmount = mutationDist(rndNumGenerator)*(max-min);
            paramVals[i] = addOrSubtract ? crossParamVals[i] + mutateAmount : crossParamVals[i] - mutateAmount;

            //Checks for error
            if(paramVals[i] > paramRanges.rows[i].second[1] || paramVals[i] < paramRanges.rows[i].second[0]){
                throw runtime_error("Mutation outside of parameter range for parameter " + paramNames[i]);
            }

        }

        //Chance of discrete mutation is low, but if it happens the discrete parameter is randomly 
        uniform_real_distribution discreteMutationDist(0.0, 1.0);
        for(int i = 0; i < nDiscrete; i++){

            int nChoices = discreteTables[i].rows.size();
            uniform_int_distribution discreteParamDist(0, nChoices-1);

            double mutationChance = 0.5*(mutationDist.max() - mutationDist.min());

            if(discreteMutationDist(rndNumGenerator) > mutationChance){
                discreteVals[i] = discreteParamDist(rndNumGenerator);
            }else{
                discreteVals[i] = crossDiscreteVals[i];
            }
        }

    }


    //MULEaircraft.plot(500, 500, paramVals, discreteVals, 50.0);

    //Quit SDL
    //SDL_Quit();

    MPI_Finalize();

    return 0;
}