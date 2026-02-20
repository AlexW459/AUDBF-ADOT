# AUDBF-ADOT
The Aircraft Design and Optimisation Toolbox is a library of code developed for use by the AUDBF to optimise aircraft designs

Requirements:
- openfoam2512
- glm3
- SDL2
- openmpi (other distributions of mpi may require some modifications to the code)
- slurm (other schedulers will require modifications to the slurm batch scripts)

Notes on dependencies: 
I have built and tested this program on an AWS cluster, which uses Oneapi to integrate MPI with Slurm. More information here: https://blog.ronin.cloud/how-to-enable-intel-mpi/ . This requires the command "source /opt/intel/oneapi/setvars.sh" to be called before compiling or running the program, in order to initialise the environment. If using a different HPC service, this might not be necessary.
I have found that Openfoam does not install correctly using apt, and it must be installed using the steps described here: https://openfoam.org/download/13-ubuntu/ , except with all mentions of openfoam13 replaced with openfoam2512.
Installing the package "libglm-dev" is sufficient to install glm.
I have found that SDL2 does not install correctly on a cluster using apt, and the best option is to follow the steps listed here: https://wiki.libsdl.org/SDL2/Installation to clone and build the library. Make sure to install all of the dependencies first: https://wiki.libsdl.org/SDL3/README-linux#build-dependencies .


Code functionality:
1. A tetrahedral mesh of each part of the aircraft is constructed according to a set of parameters describing the shape of the aircraft (initially random within defined bounds)
   - User-defined functions describe the construction of the aircraft, taking the values of the parameters as inputs. One set of functions describes the arrangement of points to create 2-dimensional shapes, such as airfoils or beam cross-sections, while another set of functions describes how these shapes are extruded into 3-dmensional parts, as well as how the parts are translated and rotated into place. All parts exist within a tree structure, where the transformations of lower parts in the tree occur within the reference frame of their parents part, which is then moved into place according to its own transformations.
   - By triangulating the 2d part cross-sections, the extrusions can be modelled as a set of triangular prisms (possibly irregular prisms if the extrusion is scaled or translated      along its length). By splitting each rectangular face of the prisms into two triangular faces, each prism becomes 6 tetrahedrons.
2. The tetrahedral meshes can be used to estimate the mass, centre of mass, and moment of inertia tensor of the aircraft
3. A triangular mesh of the surface of the aircraft is constructed according to the same set of parameters
   - A signed distance field is calculated in the bounding box of each part, by translating and scaling a meshgrid of coordinates according to the translation and scaling that occurs along its length (extruded in the +z direction), and then finding the signed distance of each of those points to the 2d cross-section. The signed distance field produced by this method is not entirely accurate, but it has the same gradient as the correct field at the location of the surface (when the field is zero).
   - A triangular mesh can be produced from this field using the marching cubes algorithm.
4. The triangular mesh is used to perform aerodynamic simulations at a range of angles of attack and a range of elevator deflections. The parameters measured are theforce on the aircraft excluding the horizontal stabiliser, the force on the horizontal stabiliser, and the velocity of the air in the region in front of the horizontal stabiliser.
   - The Openfoamv2512 API is used to perform all simulations in this application. The mesh is produced by using snappyHexMesh to overlay the triangular mesh onto a background mesh produced by blockMesh
   - Aerodynamic simulations are performed by first initialising fields using potentialFoam, and then running pimpleFoam for however much time is specified
5.  The configurations of the aircraft that lead to trim flight are determined based on the simulation data
   - The velocity and forces on the aircraft for a range of different motor throttles for each of the configurations is estimated, and the results of these estimations can be described as a 3-dimensional field of values, where the axes represent the values of elevator deflection, angle of attack and motor throttle, and the values represent the net vertical force on the aircraft. The same can be done to find a field representing the pitching torque on the aircraft for the different configurations.
   - The marching cubes algorithm is used to produce a surface from each of these fields that represent the points at which each of the forces are zero.
   - An evenly distributed sample of points at the intersection between these two surfaces is found, which represent a series of configurations of the aircraft the ainvolve zero net vertical force, and zero net pitching torque, which are the conditions necessary for trim flight.
   - Around each of these configurations, a series of additional simulations is performed to estimate the rate of change of pitching torque with respect to angle of attack, the pitch damping stability derivative (M_q) and the oscillation frequency
   - The best of these configurations is found using a user-specified function that takes into account any of the following: angle of attack, elevator deflection and motor throttle of the specified configuration, estimated velocity, net aerodynamic forces, and the three values estimated in the previous step.
6. The score of the best configuration for each design is found using the aformentioned function, and this score can be optimised for
   - Optimisation is performed using a genetic algorithm, which uses a weighted random based on the score assigned to each design to find the parents of each design of the next generation.
   - Each parameter of a child design is found by picking the value of that parameter from either of the parents, decided randomly, and then applying a random mutation. The  random mutation is found by getting a random value centred around zero using a normal distribution and flattening it to a value between -1 and 1 using a sigmoid function. This value then determines the proportion of the maximum change (positive or negative) that is made to the parameter, with the maximum change determined by the parameter bounds.

Parallelisation:
- Testing of each aircraft design is done in parallel
- Openfoam simulations performed on each design are themselves run using multiple processes

How to use:
- MULEplaneModel is provided as an example so that other 
- When running the program, the first argument is the number of aircraft designs to test in parallel, and the second argument is the number of processes per design.
- Define ranges of each of the parameters that are used to define the aircraft in a csv file, using the formatting shown in the example.
- Specify the different batteries and motors that are available in 2 more csv files. The thrust, mass, 
- Define profile (2d cross-sections) and extrusion functions for the aircraft. Profiles are defined by entering a vector of 2d points (glm::dvec2) that describe the outline of the profile, and possibly specifying an inset thickness if a hollow part is desired. Extrusions are defined by entering a vector of values describing the z positions of each of   the profiles in the extrusion, and two vectors of glm::dvec2 describing the translation and scaling of each of the profiles. The z positions of the profiles can extend in either the positive or negative z direction.
- The plot function can be used to view profiles or extrusions and the entire aircraft. Use profile.plot(width, height) after creating it to view it. When the program is run, a window will display the profile until the escape key is pressed. To plot an extrusion, you will need to temporarily copy the code that generates the profile used by that extrusion into the extrusion function, so the profile can be provided as the third argument to the extrusionData.plot() function. To plot the entire aircraft, use aircraft.plot(width, height, paramvals, discretevals, resolution) after all of the parts have been added to the aircraft in main().
- If using the plot function, make sure to uncomment the lines at the beginning of main() that initialises SDL subsystems.
- When designing the aircraft, it is useful to set the constant RUN_OPTIMISATION_LOOP to false. This quits the program from progressing past the stage of constructing the first generation of meshes. In addition, the triangular mesh from each member of the generation is produced as an OBJ file so that it can be more easily double-checked.
- Parameter values and names are passed to each profile and extrusion function, and can be used to modify how the parts are constructed.
- Any additional parameters that are derived from the basic parameters can be defined in the derivedParamFunc, and those parameters will also be passed to the profile and extrusion functions.
- Parts that include an additional mass, such as a motor or battery, can be defined using a different extrusion constructor function. Refer to extrusion.h to see all of the different constructors. Propellor diameter and pitch is measured in inches, because those are the dimensions that propellors are advertised and purchased in. All other measurements that are utilised by the program should be given in SI units. The unit system of parameters that are only used to construct parts is arbitrary as the use of those parameters is defined by the user, but the dimensions provided to the program should be measured in metres.
- Control surfaces also require a different constructor, as does the part that comprises the horizontal stabiliser, which must be specified. There should also be a part that is a   control surface but is labelled as a horizontal stabiliser. This is the elevator. Currently the code only supports one elevator flap part.
- Include the header file for these functions in the main.h and aircraft.h file, and add the cpp file with the definitions to the makefile dependencies.
- Add sufficient calls to aircraft.addPart to add all extrusions to the aircraft object. For each extrusion, give the part a name and specify its parent part (aside from the topmost part, which does not have a parent obviously)
- Define all necessary constants (refer to example MULEplaneModel/constant.h) in a header file and include the file in aircraft.h. Also include in meshWindow.h in order to use the plot function
- The RunProgram.sh slurm batch script should be modified to request the desired resources.
