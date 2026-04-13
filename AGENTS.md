# Repository Guidelines

## Project Structure & Module Organization
`host/src` contains the CLI and CPU reference implementation. `host/test` holds mock-backed unit tests plus `hw/` smoke and board-run scripts. `driver/uio` contains the userspace UIO/u-dma-buf library and generated register headers under `generated/`. `fpga/hls/tile` and `fpga/hls/stream` contain the two kernel implementations and test benches; `fpga/tcl` drives Vitis/Vivado; `fpga/pynq` stores overlay artifacts and notebooks. `petalinux/` contains the Dockerized EDF/PetaLinux flow, `driver/dts/` contains device-tree snippets, and `docs/` contains design notes. Treat `fpga/build/`, `edf/`, `.Xil/`, and generated IP repos as build output, not hand-edited source.

## Build, Test, and Development Commands
Use the root `Makefile` as the entry point:

- `make driver` builds `driver/uio/libvi_sweep.{a,so}`.
- `make host` builds `host/vi_cli`.
- `make test-host` builds and runs all mock host tests.
- `make test-hw` runs hardware smoke/integration scripts; requires `VI_TARGET_HOST`.
- `make csim KERNEL=tile` or `make hls KERNEL=stream` runs HLS simulation/export for the selected kernel.
- `make bitstream KERNEL=tile` runs the full Vivado bitstream flow.
- `make sync-hw-header KERNEL=stream` refreshes generated register headers in `driver/uio/generated/`.
- `make edf-docker`, `make edf-setup XSA=/path/to/design.xsa`, and `make edf-build MACHINE=ultra96v2-vi` drive the Linux image flow.

## Coding Style & Naming Conventions
Follow the existing C/C++ style: 4-space indentation, braces on the same line, and short, direct comments only where needed. Keep compiler settings warning-clean under `-Wall -Wextra -Werror`. Use snake_case for functions and variables (`transitions_compute`, `max_sweeps`), `*_t` for typedefs, and `ALL_CAPS` for macros/constants. Name new tests `test_<feature>.c` and keep kernel-specific files under `fpga/hls/<kernel>/src` or `tb`.

## Testing Guidelines
Host tests are plain C executables built from `host/test/test_*.c`; they should pass through `make test-host` without FPGA access. Hardware checks live in `host/test/hw/` and currently use `bash`, `ssh`, and `scp` to run `host/vi_cli` on the Ultra96 target. When changing HLS interfaces or register maps, run the relevant `make csim` target and then `make sync-hw-header KERNEL=<tile|stream>`.

## Commit & Pull Request Guidelines
Recent history uses scoped, imperative subjects such as `fix(fpga): ...`, `docs: ...`, `refactor(fpga): ...`, and `chore: ...`. Keep that format and scope commits to one subsystem when possible. PRs should state which area changed (`host`, `driver`, `fpga`, or `petalinux`), list the exact commands run, note the kernel/board tested, and include logs or screenshots only when the change affects hardware bring-up, notebooks, or generated artifacts.
