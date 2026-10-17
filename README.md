# AUDBF-ADOT
The Aircraft Design and Optimisation Toolbox is a program developed for use by AUDBF for aircraft design optimization

Requirements:
- OpenFOAM v13
- glm3
- SDL2 (Not strictly required as its inclusion can be disabled, but it is very useful for verifying designs)
- openmpi (other distributions of mpi may require some modifications to the code)

Tested to run on Ubuntu 24.04

## Usage ##

- Code is contained in Final_ADOT_program subdirectory. Enter this directory and run the AllMake.sh script to build. Can choose to build with or without SDL, which enables the ability to visualise profiles. To enable, uncomment the #define USE_SDL line in compile_config.h, and ensure -lSDL2 is in the COMPILEFLAGS variable defined in in AllMake.sh. Ensure you are building the program in the directory in which you intend for it to be located, as it will not function correctly if you move it later (without rebuilding).
- Cube, airfoilTilt and MULEplaneModel are provided as example cases. To create an example case, you will need a single C++ file and optionally a header file. Include "ADOT.h" in the relevant file and define the function: extern "C" testModel constructModel(). This contains code that sets up case by creating a testModel object with all necessary information, including the optimisation parameter bounds, the values of velocity, gravity and control surface positions for each simulation, and the functions used to construct the model to be tested.
- Compile your case files with -shared and -fpic, and the ADOT binary can now be called with the path to the .so file given as the first argument. The -writeObjs option causes the program to exit before performing any simulations, and instead writes obj files of the generated model to the current directory. It can be used to verify that you have constructed your model correctly
