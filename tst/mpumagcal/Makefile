TARGET = mpumagcal

CC = gcc

CFLAGS += -Wall -Wextra -pedantic -Wshadow -std=c11
CFLAGS += -D_GNU_SOURCE
CFLAGS += -DMPU9250
CFLAGS += -DDENOISE
CFLAGS += -DINITDEFAULT
CFLAGS += -DIOCTLINIT
#CFLAGS += -DSETBIAS
#CFLAGS += -DSETXG
#CFLAGS += -DMAGBIAS
CFLAGS += -DACCCALMAN
CFLAGS += -DGYROCALMAN

LDFLAGS += -litimer -li2c-smbus -lmpu -lrt

SOURCE = $(wildcard *.c)
OBJS = $(patsubst %.c,%.o,$(SOURCE))

$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

.PHONY: clean
clean:
	@rm -rf *.o
	@rm -rf $(OBJS)
	@rm -rf $(TARGET)

