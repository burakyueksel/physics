SRC_DIR		:= src
OBJ_DIR		:= obj
BIN_DIR		:= build
TEST_DIR	:= tests
LIB_DIR		:= include


EXE			:= $(BIN_DIR)/main
SRC			:= $(wildcard $(SRC_DIR)/*.c)
#SRC	:= $(SRC_DIR)/testMath.c
OBJ			:= $(SRC:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
LIB_PATH	:= -I$(LIB_DIR)

CFLAGS		:= -Wall -O2
CFLAGS 		:= -c -Wall $(LIB_PATH)
LDFLAGS		:= -Llib
LDLIBS 		:= -lm

.PHONY: all clean

all: $(EXE)

$(EXE): $(OBJ) | $(BIN_DIR) $(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR) $(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BIN_DIR) $(OBJ_DIR): mkdir -p $@

clean:
	@$(RM) $(BIN_DIR)/*.o
	@$(RM) $(OBJ_DIR)/*.o
	@$(RM) $(TEST_DIR)/*.o

test:
	$(MAKE) -C $(TEST_DIR)

-include $(OBJ:.o=.d)