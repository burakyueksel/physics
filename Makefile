TEST_DIR := tests

SRC = physics.c
OBJ = $(SRC:.c=.o)
BIN = ./physics
CFLAGS = -Wall -O2

all: $(BIN)

$(BIN): $(OBJ)
	gcc $^ $(CFLAGS) -o $@

%.o: %.c
	gcc -c $^ $(CFLAGS) -o $@

clean:
	$(RM) -r $(BIN)
	$(RM) -r *.o
	$(RM) $(TEST_DIR)/*.o

test:
	$(MAKE) -C $(TEST_DIR)