# Makefile for the Phyiscs
# Author: Burak Yueksel <mail.burakyueksel@gmail.com>


# Compiler and flags
CC = gcc
CFLAGS = -Wall -O2
LDFLAGS=

# Source files
SRC_DIR = src
SRCS = $(wildcard $(SRC_DIR)/*.c)

# Header files
INC_DIR = include
INCS = $(wildcard $(INC_DIR)/*.h)

# Object files
OBJ_DIR = obj
OBJS = $(SRCS:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

# Executable
TARGET = main

# Test files
TEST_DIR := tests

# Include directory
CFLAGS += -I$(INC_DIR)

# Default target
all: $(TARGET)

# Link object files to create executable

# We use the following example:
#./main: main.o
#	gcc main.o -o ./main
# which generates ./main instead of a.out when running gcc
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS)

# Compile source files to object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c $(INCS)
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean

clean:
	rm -f $(OBJS) $(TARGET) *~
	$(RM) $(TEST_DIR)/*.o
	$(RM) $(TEST_DIR)/*~

test:
	$(MAKE) -C $(TEST_DIR)