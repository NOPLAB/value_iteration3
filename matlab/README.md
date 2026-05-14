# MATLAB VI Workspace

MATLAB-side workspace for value-iteration algorithm validation, benchmarking,
CPU prototyping, FPGA HDL Coder flows, and future CUDA-oriented experiments.

## Required Toolboxes

- MATLAB R2024b+
- Simulink
- HDL Coder
- HDL Verifier
- Fixed-Point Designer
- SoC Blockset
- Zynq UltraScale+ MPSoC support package (or Ultra96-V2 BSP)

## Quick Start

```matlab
% 1. Run the matlab.unittest suite (no toolboxes needed beyond base MATLAB)
run_matlab_tests

% 2. Fixed-point analysis (requires Fixed-Point Designer)
setup_matlab_paths('validation'); fp_config

% 3. HDL cosimulation (requires HDL Verifier + Vivado Xsim)
setup_matlab_paths('validation'); cosim_tb

% 4. Export packaged IP for the repo Vivado flow
setup_matlab_paths('fpga-export'); export_repo_ip
```

## Directory Structure

```
matlab/
├── src/
│   ├── common/    Shared helpers and constants
│   ├── cpu/       CPU/reference/frontier implementations
│   ├── fpga/      FPGA-mimic and SoC kernel logic
│   ├── cuda/      Future CUDA/MEX-backed experiments
│   └── shared/    Shared low-level utilities such as bitboards
├── workflows/
│   ├── benchmarks/
│   └── validation/
├── platforms/
│   └── fpga/      Board support, model generation, export, and SoC flows
├── artifacts/     Generated outputs, cached build products, benchmark CSVs
├── resources/     MATLAB project metadata
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

1. Run `workflows/validation/fixedpoint/fp_config.m` to analyze dynamic range
2. Open Simulink model -> Fixed-Point Tool -> apply proposed types
3. Re-run `run_matlab_tests` to verify zero-error conversion
4. Target bit widths: value=16, penalty=16, offset=8 (matching HLS)

### Phase C: HDL Generation and Cosimulation

1. Open `platforms/fpga/model/vi_sweep_stream_matlab.slx` in Simulink
2. HDL Workflow Advisor -> Generate HDL
3. Run `workflows/validation/cosim/cosim_tb.m` with Xsim backend
4. Verify cycle-accurate match against golden MATLAB output

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
make matlab-cosim      # Run cosimulation
make matlab-bitstream  # Build Ultra96-V2 bitstream from the exported IP
make matlab-bench      # Compare reference/frontier/fpga-mimic CPU paths
make matlab-bench-codegen  # Compare MATLAB vs MATLAB Coder MEX timings
```

## Constants

All constants are defined in `src/common/vi_params.m` and match
`fpga/hls/stream/src/vi_stream_types.h`. See the design spec at
`docs/superpowers/specs/2026-04-13-matlab-hdl-coder-streaming-design.md`.
