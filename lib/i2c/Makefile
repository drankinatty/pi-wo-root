# == Options ==
# 1. make = compile library
# 2. make install = install library to filesystem may need sudo)
# 3. make clean = deletes .o files generated by step 1 from build directory
# 4. make uninstall  = uninstalls library (may need sudo)
# =============

# Where you want it installed
PREFIX = /usr
# where to place the libray
LIBDIR = $(PREFIX)/lib
# library name
LIB=libi2c-smbus
# soname version
SONAME = $(LIB).so.1
# shared library name
LIBNAME = $(SONAME).0

MD  = mkdir
SRC = src
OBJ = obj
SRCS = $(wildcard $(SRC)/*.c)
OBJS = $(patsubst $(SRC)%.c,  $(OBJ)/%.o, $(SRCS))

CC = gcc
CFLAGS = -march=native -mtune=native -mcpu=native -Iinclude/
CFLAGS += -Wall -Wextra -pedantic -Wshadow -std=c11
CFLAGS += -D_GNU_SOURCE
LDFLAGS =

# make all
# reinstall the library after each recompilation
all: clean pre-build lib

pre-build:
	@echo
	@echo "*****************"
	@echo "[MAKE LIBRARY]"
	@echo
	$(MD) -vp $(OBJ)

# Make the library
lib: 	$(OBJS)
	$(CC) -shared -Wl,-soname,$(SONAME) $(CFLAGS) $(LDFLAGS)  -o ${LIBNAME} $^

# Library parts
$(OBJ)/%.o: $(SRC)/%.c
	$(CC) -Wall -Wextra -Wshadow -fPIC -c $(CFLAGS) $< -o $@

# Install the library to LIBPATH
install:
	@echo
	@echo "*****************"
	@echo "[INSTALL LIBRARY]"
	@if ( test ! -d $(PREFIX)/lib ) ; then mkdir -vp $(PREFIX)/lib ; fi
	@install -vm 0755 ${LIBNAME} ${LIBDIR}
	@ln -svf ${LIBDIR}/${LIBNAME} ${LIBDIR}/${LIB}.so.1
	@ln -svf ${LIBDIR}/${LIBNAME} ${LIBDIR}/${LIB}.so
	@ldconfig
	# @rm -rvf ${LIB}.*
	@echo "*****************"
	@echo
	@echo "[INSTALL LIBRARY HEADERS]"
	@if ( test ! -d $(PREFIX)/include ) ; then mkdir -p $(PREFIX)/include ; fi
	@cp -vf  include/$(LIB).h $(PREFIX)/include
	@echo "[DONE!]"

# Uninstall the library
uninstall:
	@echo "******************"
	@echo "[UNINSTALL LIBRARY]"
	@rm -vf ${LIBDIR}/${LIB}.*
	@ldconfig

	@echo "[UNINSTALL LIBRARY HEADERS]"
	@rm -rvf  $(PREFIX)/include/$(LIB).h
	@echo "[DONE!]"

# clear build files
clean:
	@echo "******************"
	@echo "[CLEAN OBJECT FILES]"
	#rm -rvf $(OBJ)/*.o ${LIB}.*
	rm -rvf $(OBJ) ${LIB}.*
	@echo "[DONE!]"
