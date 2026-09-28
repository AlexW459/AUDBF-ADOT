#pragma once

// Uncomment to compile with SDL, which enables plotting of mesh
//#define USE_SDL

//#define OPENFOAM_SOURCE "$HOME/opt/OpenFOAM-13/etc/bashrc"
#define OPENFOAM_SOURCE "/opt/openfoam13/etc/bashrc"

//Runs optimisation loop rather than quitting following first simulation
#define RUN_OPTIMISATION_LOOP true
//Enables writing produced obj files to the root directory. Useful for double checking aircraft designs
#define WRITE_OBJS_TO_ROOT false


//Distance from screen to object being viewed. Used to move in or out to view objects 
//of different sizes
#define MESH_WINDOW_DIST_TO_SCREEN 0.3
