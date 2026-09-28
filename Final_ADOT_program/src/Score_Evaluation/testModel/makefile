

# Compiler
CXX = mpicxx

# Source files
SRCS = $(wildcard *.cpp)

# Object files
OBJS = $(SRCS:.cpp=.o)

all: $(OBJS)

# Rule to compile .cpp files into .o files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(COMPILE_FLAGS)

# Clean rule to remove generated files
clean:
	rm -f $(OBJS)
