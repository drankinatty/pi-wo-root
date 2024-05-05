## Gavin Lyons 06-2021
# Makefile to build example program for SSD1306_OLED_RPI library.
# URL: https://github.com/gavinlyonsrepo/SSD1306_OLED_RPI
# Rewrite removing Broadcom bcm2835 library by David C. Rankin, JD,PE
# URL: https://github.com/drankinatty
# ************************************************
# User SRC directory Option Section
# Pick on example "SRC" dir path and ONE Only.
# Comment out the other's.

#SRC = src/HELLO_WORLD
#SRC = src/BITMAP
#SRC = src/CLOCK_DEMO
#SRC = src/OLED_FUNCTIONS
SRC = src/SPEED_TEST
#SRC = src/TEXT_GRAPHICS
#SRC = src/I2C_TEST
# ************************************************

#APP = test-hello
#APP = test-bitmap
#APP = test-clock
#APP = test-oled-functions
APP = test-speed
#APP = test-text-graphics
#APP = test-i2c

# ********************
# Options
# 1. make
# 2. make clean
# 3. make run
# ********************

CC = g++

BIN = ./bin
OBJ = obj
TARGET = $(BIN)/$(APP)
MD = mkdir

CFLAGS = -Iinclude/ -c -Wall -Wextra -std=gnu++17
LDFLAGS = -lrt -lssd1306
SRCS = $(wildcard $(SRC)/*.cpp)
OBJS = $(patsubst $(SRC)/%.cpp,  $(OBJ)/%.o, $(SRCS))

all: clean pre-build $(TARGET)

run:
	sudo $(TARGET)

pre-build:
	@echo '*******'
	@echo '[START!]'
	@echo 'Building: ' $(SRC)
	$(MD) -vp $(BIN)
	$(MD) -vp $(OBJ)


$(TARGET): $(OBJS) $(OBJ)
	@echo '[MAKE EXE FILE!]'
	$(CC)  $(OBJS) -o $@ $(LDFLAGS)
	@echo '[DONE!]'

$(OBJ)/%.o :  $(SRC)/%.cpp   $(OBJ)
	@echo '[MAKE OBJECT FILE!]'
	$(CC) $(CFLAGS) $< -o  $@

clean:
	@echo '*******'
	@echo '[CLEANUP!]'
	rm -rvf $(BIN) $(OBJ) $(TARGET)
	@echo '[DONE!]'
