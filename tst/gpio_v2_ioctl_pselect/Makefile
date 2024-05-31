TARGET = gpio_v2-ioctl-pselect

CC = gcc

CFLAGS += -Wall -Wextra -pedantic -Wshadow -std=c11
CFLAGS += -D_GNU_SOURCE
#CFLAGS += -DDEBUG

LDFLAGS += -pthread

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

