# libvi_sweep — C library for vi_sweep FPGA IP (UIO + u-dma-buf)

CC      ?= gcc
AR      ?= ar
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O2 -fPIC -Iinclude -I. -Igenerated
LDFLAGS := -lpthread

SRC_COMMON := libvi_sweep.c vi_device_mock.c
SRC_LINUX  := vi_device_linux.c
OBJ_COMMON := $(SRC_COMMON:.c=.o)
OBJ_LINUX  := $(SRC_LINUX:.c=.o)

HEADERS := libvi_sweep.h vi_device.h

LIB_STATIC := libvi_sweep.a
LIB_SHARED := libvi_sweep.so

.PHONY: all clean sync-hw-header mock-only

all: $(LIB_STATIC) $(LIB_SHARED)

# Host-only build (no Linux UIO, for unit tests)
mock-only: CFLAGS += -DVI_MOCK_ONLY
mock-only: libvi_sweep_mock.a

libvi_sweep_mock.a: $(OBJ_COMMON)
	$(AR) rcs $@ $^

$(LIB_STATIC): $(OBJ_COMMON) $(OBJ_LINUX)
	$(AR) rcs $@ $^

$(LIB_SHARED): $(OBJ_COMMON) $(OBJ_LINUX)
	$(CC) -shared -o $@ $^ $(LDFLAGS)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

# Copy HLS-generated register header into generated/
# Usage: make sync-hw-header KERNEL=tile   (or KERNEL=stream)
KERNEL ?= tile
IP_REPO := ../../fpga/vivado/ultra96v2/ip_repo_$(KERNEL)

ifeq ($(KERNEL),stream)
  HLS_HW_HEADER := $(IP_REPO)/drivers/vi_sweep_stream_v1_0/src/xvi_sweep_stream_hw.h
  GEN_HEADER    := generated/xvi_sweep_stream_hw.h
else
  HLS_HW_HEADER := $(IP_REPO)/drivers/vi_sweep_v1_0/src/xvi_sweep_hw.h
  GEN_HEADER    := generated/xvi_sweep_hw.h
endif

sync-hw-header:
	@test -f $(HLS_HW_HEADER) || { echo "HLS header not found: $(HLS_HW_HEADER)"; exit 1; }
	install -D $(HLS_HW_HEADER) $(GEN_HEADER)
	@echo "Synced $(GEN_HEADER). Review diff with: git diff $(GEN_HEADER)"

clean:
	rm -f *.o *.a *.so
