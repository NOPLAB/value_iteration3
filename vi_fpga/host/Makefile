# host: vi_cli + unit tests

CC      ?= gcc
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O2 \
           -Isrc -I../driver/uio -I../fpga/hls/vi_sweep/src
LDFLAGS :=

# --- vi_cli ---
CLI_SRC := src/vi_cli.c src/map_pgm.c src/penalty.c src/transitions.c src/vi_reference_c.c
CLI_OBJ := $(CLI_SRC:.c=.o)
CLI_BIN := vi_cli

# Link against the Linux libvi_sweep (static)
LIBVI := ../driver/uio/libvi_sweep.a
LIBVI_MOCK := ../driver/uio/libvi_sweep_mock.a

# --- Unit tests ---
TEST_SRC := test/test_map_pgm.c \
            test/test_penalty.c \
            test/test_transitions.c \
            test/test_vi_run_mock.c \
            test/test_action_table.c \
            test/test_reference_eq.c
TEST_BINS := $(TEST_SRC:.c=)

.PHONY: all clean test-host test-hw cli cli-mock

all: cli

cli: $(CLI_BIN)

$(CLI_BIN): $(CLI_OBJ) $(LIBVI)
	$(CC) -o $@ $^ $(LDFLAGS) -lpthread -lm

# Mock-only CLI build (no Linux ops, for host smoke test)
cli-mock: CFLAGS += -DVI_MOCK_ONLY
cli-mock: $(LIBVI_MOCK)
	$(CC) $(CFLAGS) -o vi_cli_mock $(CLI_SRC) $(LIBVI_MOCK) -lm

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# --- Host tests (mock only, no libpthread UIO code) ---
test-host: $(LIBVI_MOCK) $(TEST_BINS:=.run)
	@echo "=== All host tests PASSED ==="

$(LIBVI_MOCK):
	$(MAKE) -C ../driver/uio mock-only

test/%: test/%.c src/map_pgm.c src/penalty.c src/transitions.c src/vi_reference_c.c $(LIBVI_MOCK)
	$(CC) $(CFLAGS) -DVI_MOCK_ONLY -DVI_TEST -o $@ $^ -lm

%.run: %
	@echo "--- Running $< ---"
	@./$<

# --- HW integration tests ---
test-hw:
	@test -n "$$VI_TARGET_HOST" || { echo "set VI_TARGET_HOST"; exit 1; }
	bash test/hw/run_smoke.sh
	bash test/hw/run_big.sh

clean:
	rm -f src/*.o test/*.o $(CLI_BIN) vi_cli_mock $(TEST_BINS)
