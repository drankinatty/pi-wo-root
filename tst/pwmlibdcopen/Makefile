TARGET = pwmlibdcopen

CC = gcc

CFLAGS += -Wall -Wextra -pedantic -Wshadow -std=c11
CFLAGS += -D_GNU_SOURCE
# enable debug output
#CFLAGS += -DDEBUG
# disable ANSI escapes
#CFLAGS += -DNOANSI

LDFLAGS += -lpwm

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

