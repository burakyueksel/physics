LIBS	:= -I"../"
SRC = testMatrix.c
OBJ = $(SRC:.c=.o)
BIN = ./testMatrix
CFLAGS = -Wall -O2
CFLAGS = $(LIBS)

all: $(BIN)

$(BIN): $(OBJ)
	gcc $^ $(CFLAGS) -o $@

%.o: %.c
	gcc -c $^ $(CFLAGS) -o $@

clean:
	$(RM) -r $(BIN)
	$(RM) -r *.o