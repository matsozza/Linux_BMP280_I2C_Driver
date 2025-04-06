VERSION=1:0:0

# Compiler and Flags
CROSS_COMPILE=aarch64-linux-gnu-
CC=$(CROSS_COMPILE)gcc
CFLAGS=-Wall -Wextra -ggdb3 -I./include -pthread
LDFLAGS=-static -L./lib 
LIBS=-lgpiod

# Directories
SRC_DIR:=./src
OBJ_DIR:=./obj
LIB_DIR:=./lib
BIN_DIR:=./bin

# Filenames
BIN_NAME:=bmp280

# File Names
SRC_FILES:=$(wildcard $(SRC_DIR)/*.c)
OBJ_FILES:=$(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SRC_FILES) )
BIN_FILES:=$(BIN_DIR)/$(BIN_NAME)

# Rule to compile .c files into .o files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@echo "\n------------------------------------------------"
	@echo "Compiling source file to object file" | fold -w 48
	@echo "SOURCE: " $< | fold -w 48
	@echo "OBJECT: " $@ | fold -w 48
	@echo "------------------------------------------------"
	@mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Rule to compile test file
$(BIN_FILES): $(OBJ_FILES)
	@echo "\n------------------------------------------------"
	@echo "Compiling source files to executable" | fold -w 48
	@echo "SOURCE: " $^ | fold -w 48
	@echo "TEST: " $@ | fold -w 48
	@echo "------------------------------------------------"
	@rm -rf $(BIN_FILES)
	@mkdir -p $(BIN_DIR)
	$(CC) $^ -o $@ $(LDFLAGS) $(LIBS) 

# Default rule to compile libraries
all: $(BIN_FILES)

# Rule to compile test file
test: $(BIN_FILES)
	@echo "\n------------------------------------------------"
	@echo "Sending test files to the TARGET" | fold -w 48
	@echo "------------------------------------------------"
	ssh matheus@rpi.local 'rm -rf *; mkdir -p lib; mkdir -p bin; mkdir -p src'
	scp $(BIN_FILES) matheus@rpi.local:~/$(BIN_FILES)
	# scp $(SRC_FILES) matheus@rpi.local:~/$(SRC_DIR)
	@echo "\n------------------------------------------------"
	@echo "DONE!" | fold -w 48
	@echo "------------------------------------------------"

# Clean rule to remove object files and binaries
clean:
	@echo "\n------------------------------------------------"
	@echo "Cleaning all previous build files     "
	@echo "------------------------------------------------"
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# Rule for phony BIN_FILESs (not files)
.PHONY: all clean test