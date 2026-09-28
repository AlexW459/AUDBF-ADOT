

# Compiler
CXX = mpicxx

# Target executable
TARGET = ../ADOT

#Source files
SRCS = $(wildcard *.cpp) $(wildcard */*.cpp) $(wildcard */*/*.cpp) $(wildcard */*/*/*.cpp)

# Object files
OBJS = $(SRCS:.cpp=.o)


# First compiles object files, then links them together
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(TARGET) $(COMPILEFLAGS)
	


# Rule to compile .cpp files into .o files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(COMPILEFLAGS)

# Rule to run the executable
run: mpirun -n 1 $(TARGET)

# Clean rule to remove generated files./
clean:
	rm -f $(OBJS) $(TARGET)

single:
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(COMPILEFLAGS)
