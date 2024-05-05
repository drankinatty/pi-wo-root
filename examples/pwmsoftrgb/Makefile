TARGET = pwmsoftrgb

CC = gcc

CFLAGS += -Wall -Wextra -pedantic -Wshadow -std=c11 -Ofast
CFLAGS += -D_GNU_SOURCE
#CFLAGS += -DDEBUG
#CFLAGS += -DUSESLEEP

LDFLAGS += -lpwmsoft -litimer -pthread -lrt

SOURCE = $(wildcard *.c)
OBJS = $(patsubst %.c,%.o,$(SOURCE))

all:	clean $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

.PHONY: clean
clean:
	@rm -rf *.o
	@rm -rf $(OBJS)
	@rm -rf $(TARGET)

