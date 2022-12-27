TEST_DIR := tests

CC = gcc
OBJFILES = main.o physics.o
TARGET = ./main
CFLAGS = -Wall -O2
LDFLAGS=

all: $(TARGET)

# We use the following example:
#./main: main.o
#	gcc main.o -o ./main
# which generates ./main instead of a.out when running gcc
$(TARGET): $(OBJFILES)
	$(CC) $(CFLAGS) $(OBJFILES) -o $(TARGET) $(LDFLAGS)

.PHONY: clean

clean:
	rm -f $(OBJFILES) $(TARGET) *~
	$(RM) -r *.o
	$(RM) -r *.out
	$(RM) $(TEST_DIR)/*.o
	$(RM) $(TEST_DIR)/*~

test:
	$(MAKE) -C $(TEST_DIR)