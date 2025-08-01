# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -Wall -g -fopenmp-simd

# Target executable
TARGET = Main

# For deleting the target
TARGET_DEL = Main.exe

# Source files
SRCS = main.cpp aircraft.cpp readCSV.cpp Mesh_Generation/profile.cpp \
Mesh_Generation/meshWindow.cpp MULEplaneModel/MULEplane.cpp

# Object files
OBJS = $(SRCS:.cpp=.o)

SDL_INCLUDE = -lSDL2


# Rule to link object files into the target executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(SDL_INCLUDE)

# Rule to compile .cpp files into .o files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to run the executable
run: $(TARGET)
	$(TARGET) 

# Clean rule to remove generated files
clean:
	rm $(OBJS) $(TARGET)


