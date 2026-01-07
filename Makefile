# Directory paths
DIR_SRC = ./src
DIR_INC = ./inc
DIR_OBJ = ./obj
DIR_TEST = ./test
DIR_LIB_INC = ./lib/include
DIR_LIB = ./lib

# Source files
SRC = $(wildcard ${DIR_SRC}/*.c)
OBJS = $(patsubst %.c,${DIR_OBJ}/%.o,$(notdir ${SRC}))
DEPS = $(OBJS:.o=.d)

# Targets
TARGET = ads1256dac8532

# Compiler
CC = gcc

# Add these variables for standard system paths
SYS_LIB_PATH = /usr/local/lib
SYS_INC_PATH = /usr/local/include

# Libraries and library search path
LIB = -L$(SYS_LIB_PATH)  -lm -lwiringPi 

# Compilation flags
DEBUG = -g -O0 -Wall
CFLAGS += $(DEBUG) -I$(DIR_INC) -I$(SYS_INC_PATH) -MP -MMD

# Default rule
all: ${TARGET}

# Link main target
${TARGET}: ${OBJS}
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LIB)

# Compile source files
${DIR_OBJ}/%.o : $(DIR_SRC)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean target
.PHONY: clean
clean:
	rm -f $(DIR_OBJ)/*.o $(DIR_OBJ)/*.d
	rm -f ${TARGET}

# Include dependencies
-include $(DEPS)

# Print dependencies info during make
$(info DEPS is $(DEPS))
