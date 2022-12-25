LIBS	:= -I"../"
SRC = testMath.c
OBJ = $(SRC:.c=.o)
BIN = ./testMath
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