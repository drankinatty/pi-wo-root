# program name
TARGET = mpuautocal

# compiler
CC = gcc

# compiler flags
CFLAGS += -Wall -Wextra -pedantic -Wshadow -std=c11
CFLAGS += -D_GNU_SOURCE
CFLAGS += -DMPU9250
CFLAGS += -DDENOISE
CFLAGS += -DINITDEFAULT
CFLAGS += -DIOCTLINIT
#CFLAGS += -DSETBIAS
#CFLAGS += -DSETXG
CFLAGS += -DMAGBIAS
CFLAGS += -DACCCALMAN
CFLAGS += -DGYROCALMAN

# linker flags
LDFLAGS += -li2c-smbus -lmpu -litimer -lrt

# source and object files
SOURCE = $(wildcard *.c)
OBJS = $(patsubst %.c,%.o,$(SOURCE))

# compile program from object files
$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

# compile object files from sources
%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

# clean working directory
.PHONY: clean
clean:
	@rm -rf *.o
	@rm -rf $(OBJS)
	@rm -rf $(TARGET)

