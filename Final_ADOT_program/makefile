

# Compiler
CXX = mpicxx

# Compiler flags
CXXFLAGS = -Wall -g -std=c++20 -march=x86-64 -msse2 -O3 #-fopt-info-vec-missed -fno-inline

SDL_FLAGS = #$(sdl2-config --cflags --libs)

# Target executable
TARGET = Main


# For deleting the target
TARGET_DEL = Main.exe

# Source files
SRCS = main.cpp aircraft.cpp readCSV.cpp Mesh_Generation/profile.cpp  \
	MULEplaneModel/MULEplane.cpp Mesh_Generation/extrusionGen.cpp Mesh_Generation/surfaceMeshGen.cpp \
	getForces.cpp #Mesh_Generation/meshWindow.cpp



# Object files
OBJS = $(SRCS:.cpp=.o)


# Rule to link object files into the target executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(SDL_FLAGS)

# Rule to compile .cpp files into .o files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(SDL_FLAGS)

# Rule to run the executable
run: mpirun -n 2 $(TARGET) 2


# Clean rule to remove generated files
clean:
	rm $(OBJS) $(TARGET)


