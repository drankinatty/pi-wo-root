TARGET=mpu-oled-pth

CC = g++

CFLAGS += -Wall -Wextra -pedantic -Wshadow -std=c++11
CFLAGS += -D_GNU_SOURCE
CFLAGS += -DMPU9250
CFLAGS += -DDENOISE
CFLAGS += -DINITDEFAULT
CFLAGS += -DIOCTLINIT
CFLAGS += -DSETBIAS
CFLAGS += -DDRAWFIXED

LDFLAGS += -litimer -lmpu -lssd1306 -li2c-smbus -lrt
LDFLAGS += -pthread

SOURCE = $(wildcard *.cpp)
OBJS = $(patsubst %.cpp,%.o,$(SOURCE))

$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

.PHONY: clean
clean:
	@rm -rf *.o
	@rm -rf $(OBJS)
	@rm -rf $(TARGET)

