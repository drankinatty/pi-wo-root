TARGET=mpufusion3

CC = gcc

CFLAGS += -Wall -Wextra -pedantic -Wshadow -std=c11
CFLAGS += -D_GNU_SOURCE
CFLAGS += -DMPU9250
CFLAGS += -DDENOISE
CFLAGS += -DINITDEFAULT
CFLAGS += -DIOCTLINIT

LDFLAGS += -li2c-smbus -lmpu -lsfusion -litimer -lrt -lm

SOURCE = $(wildcard *.c)
OBJS = $(patsubst %.c,%.o,$(SOURCE))

$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

.PHONY: clean
clean:
	@rm *.o -rf
	@rm $(OBJS) -rf
	@rm $(TARGET)

