# Compiler and flags
CXX = g++
CC = cc
CXXFLAGS = -std=c++11 -Wall
LDFLAGS = -lpthread

# Source files
CXX_SRC = main.cpp
C_SRC = mongoose.c

# Object files
CXX_OBJ = $(CXX_SRC:.cpp=.o)
C_OBJ = $(C_SRC:.c=.o)

# Output binary
TARGET = server

# Default target to build the program
all: $(TARGET)

# Link the object files into the executable
$(TARGET): $(CXX_OBJ) $(C_OBJ)
	$(CXX) -o $@ $^ $(LDFLAGS)

# Rule to build the C++ object file
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to build the C object file
%.o: %.c
	$(CC) -c $< -o $@

# Clean up the object files and binary
clean:
	rm -f $(CXX_OBJ) $(C_OBJ) $(TARGET)

# Make sure the target is up-to-date
.PHONY: all clean

