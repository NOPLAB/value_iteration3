# MATLAB VI Workspace

MATLAB-side workspace for value-iteration algorithm validation, benchmarking,
CPU prototyping, and FPGA HDL Coder flows.

## Required Toolboxes

- MATLAB R2024b+
- Simulink
- HDL Coder
- Fixed-Point Designer
- Zynq UltraScale+ MPSoC support package (or Ultra96-V2 BSP)

## Quick Start

```matlab
% 1. Run the matlab.unittest suite (no toolboxes needed beyond base MATLAB)
run_matlab_tests

% 2. Export packaged IP for the repo Vivado flow
setup_matlab_paths('fpga-export'); export_repo_ip
```

## Directory Structure

```
vi_matlab/
├── src/
│   ├── common/    Shared helpers and constants
│   ├── cpu/       CPU/reference/frontier implementations
│   ├── fpga/      FPGA-mimic kernel logic
│   └── shared/    Shared low-level utilities such as bitboards
├── workflows/
│   ├── benchmarks/
│   └── validation/
├── platforms/
│   └── fpga/      Board support, model generation, export
├── artifacts/     Generated outputs, cached build products, benchmark CSVs
├── run_matlab_tests.m
├── setup_matlab_paths.m
└── vi_matlab_layout.m
```

## Development Workflow

### Phase A: Floating-Point Verification

1. Edit algorithm in `src/**` (all signals are `double`)
2. Run `run_matlab_tests` to execute the MATLAB unit and integration suite
3. Iterate until all tests pass

### Phase B: Fixed-Point Conversion

1. Open Simulink model -> Fixed-Point Tool -> apply proposed types
2. Re-run `run_matlab_tests` to verify zero-error conversion
3. Target bit widths: value=16, penalty=16, offset=8 (matching HLS)

### Phase C: HDL Generation

1. Run `create_model` (`platforms/fpga/model/`) to regenerate the Simulink
   model, then HDL Workflow Advisor -> Generate HDL
2. Or run `export_repo_ip` for the packaged-IP flow used by the repo Vivado
   build

### Phase D: Bitstream and Hardware

1. Run `make matlab-bitstream` from the repository root
2. This regenerates the MATLAB HDL IP and builds `fpga/build/vi_matlab`
3. The `matlab` Vivado variant runs with `jobs=1` by default because the
   MATLAB-generated IP uses enough memory to fail on typical hosts when OOC
   synthesis is launched in parallel
4. Deploy the resulting `.bit` + `.hwh` from
   `fpga/build/vi_matlab/vi_matlab.runs/impl_1/` to Ultra96-V2

## Makefile Targets

From project root:

```bash
make matlab-sim        # Run matlab.unittest suite
make matlab-hdl        # Export MATLAB HDL IP into fpga/build/matlab_ip_repo
make matlab-bitstream  # Build Ultra96-V2 bitstream from the exported IP
make matlab-bench      # Compare reference/frontier/fpga-mimic CPU paths
```

## Constants

All constants are defined in `src/common/vi_params.m` and match
`fpga/hls/stream/src/vi_stream_types.h`. See the design spec at
`docs/superpowers/specs/2026-04-13-matlab-hdl-coder-streaming-design.md`.
