TARGET = bcm-i2c-bus-speed

CC = gcc

BUILD = "release"

CFLAGS += -Wall -Wextra -pedantic -Wshadow -std=c11
CFLAGS += -D_GNU_SOURCE

LDFLAGS += -lbcm2835

SOURCE = $(wildcard *.c)
OBJS = $(patsubst %.c,%.o,$(SOURCE))

$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)
	@[ $(BUILD) = "release" ] && strip -s $@ || true

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

.PHONY: clean
clean:
	@rm -rf $(OBJS)
	@rm -rf $(TARGET)

