//Defines various constants

#define RUN_OPTIMISATION_LOOP false
//distance from screen to object being viewed. Used to move in or out to view objects 
//of different sizes
#define MESH_WINDOW_DIST_TO_SCREEN 0.3

#define PROFILE_RESOLUTION 75.0
#define SDF_RESOLUTION 150.0

//velocity at which simulations are run
#define TEST_VELOCITY 40.0 
//k-epsilon model constants
#define TURBULENCE_INTENSITY 0.003
#define TURBULENCE_LENGTH_SCALE 0.5

//Length and timestep of simulations
#define SIMULATION_LENGTH 0.3
#define SIMULATION_DELTA_T 0.002

//Standard deviation of mutations to parameter vals (distribution is then flattened to between 0 and 1)
//using the sigmoid function
#define MUTATION_STD_DEV 0.2

//Defines ranges of and number of different configurations tested to find the positions 
//leading to trim flight. Configurations are evenly spaced
//Angle of attack (radians)
#define MIN_ALPHA -M_PI/1.0
#define MAX_ALPHA 3.0*M_PI/18.0
#define N_ALPHA 4
//Elevator deflection (radians)
#define MIN_ELEVATOR -M_PI/6.0
#define MAX_ELEVATOR M_PI/6.0
#define N_ELEVATOR 4
//Motor thrust (fraction of total)
#define MIN_THROTTLE 0.6
#define MAX_THROTTLE 1.0
#define N_THROTTLE 5

