# Makefile for the Phyiscs
# Author: Burak Yueksel <mail.burakyueksel@gmail.com>


# Compiler and flags
CC = gcc
CFLAGS = -Wall -O2
LDFLAGS=

# Source files
SRCS= main.c physics.c

# Object files
OBJFILES = $(SRCS:.c=.o)

# Test files
TEST_DIR := tests

# Executable
TARGET = ./main


# Default target
all: $(TARGET)

# Link object files to create executable

# We use the following example:
#./main: main.o
#	gcc main.o -o ./main
# which generates ./main instead of a.out when running gcc
$(TARGET): $(OBJFILES)
	$(CC) $(CFLAGS) $(OBJFILES) -o $(TARGET) $(LDFLAGS)

# Compile source files to object files
%.o: %.c
	$(CC) $(CFLAGS) -c $<

.PHONY: clean

clean:
	rm -f $(OBJFILES) $(TARGET) *~
	$(RM) -r *.o
	$(RM) -r *.out
	$(RM) $(TEST_DIR)/*.o
	$(RM) $(TEST_DIR)/*~

test:
	$(MAKE) -C $(TEST_DIR)