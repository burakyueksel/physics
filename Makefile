SRC = testMath.c
OBJ = $(SRC:.c=.o)
BIN ?= ./build
CFLAGS = -Wall -O2

all: $(BIN)

$(BIN): $(OBJ)
	gcc $^ $(CFLAGS) -o $@

%.o: %.c
	gcc -c $^ $(CFLAGS) -o $@

clean:
	$(RM) -r $(BIN)
	$(RM) -r *.o