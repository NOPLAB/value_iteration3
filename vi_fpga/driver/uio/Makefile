# libvi_sweep — C library for vi_sweep FPGA IP (UIO + u-dma-buf)

CC      ?= gcc
AR      ?= ar
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O2 -I.

SRC_COMMON := libvi_sweep.c vi_device_mock.c
SRC_LINUX  := vi_device_linux.c
OBJ_COMMON := $(SRC_COMMON:.c=.o)
OBJ_LINUX  := $(SRC_LINUX:.c=.o)

HEADERS := libvi_sweep.h vi_device.h vi_regs.h

LIB_STATIC := libvi_sweep.a

.PHONY: all clean mock-only

all: $(LIB_STATIC)

# Host-only build (no Linux UIO, for unit tests)
mock-only: CFLAGS += -DVI_MOCK_ONLY
mock-only: libvi_sweep_mock.a

libvi_sweep_mock.a: $(OBJ_COMMON)
	$(AR) rcs $@ $^

$(LIB_STATIC): $(OBJ_COMMON) $(OBJ_LINUX)
	$(AR) rcs $@ $^

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f *.o *.a *.so
