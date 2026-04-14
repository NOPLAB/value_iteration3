# matlab/src Refactoring Design

## Problem

The `matlab/src/` folder has 12 files flat in a single directory, mixing:
- Streaming pipeline stages (HDL generation targets)
- Verification reference implementations
- Shared utilities
- Parameters

The `_algo` suffix is inconsistently applied, and `_reference` suffix is redundant when the role is clear from context. It is not immediately obvious which files are HDL targets vs. verification-only.

## Solution

Reorganize `matlab/src/` into three subdirectories by role, and normalize naming by removing `_algo` and `_reference` suffixes.

## Target Structure

```
matlab/src/
├── vi_params.m                      # Shared parameters (no move)
├── pipeline/                        # HDL generation target: streaming pipeline
│   ├── vi_sweep_stream.m            # Top-level dual-CU sweep orchestration
│   ├── stream_strip.m              # X-strip processing with sliding window
│   ├── load_row.m                  # Row loading with halo
│   ├── compute_row.m               # Bellman row update
│   ├── store_row.m                 # Row storage (inner cells)
│   └── cost_of.m                   # Neighbor cost computation
├── reference/                       # Verification reference (not HDL target)
│   ├── vi_full_reference.m         # Brute-force triple-nested reference solver
│   └── compute_action_table.m      # Argmin action table computation
└── util/                            # Shared utilities
    ├── coerce_transition_model.m   # Transition model format adapter + cache
    ├── unpack_transitions.m        # Packed uint32 transition decoder
    └── make_goal_mask.m            # 3D spatial+angular goal mask builder
```

## File Rename Map

| Old Name | New Name | Change |
|----------|----------|--------|
| `vi_sweep_stream_algo.m` | `pipeline/vi_sweep_stream.m` | move + remove `_algo` |
| `stream_strip_algo.m` | `pipeline/stream_strip.m` | move + remove `_algo` |
| `load_row_algo.m` | `pipeline/load_row.m` | move + remove `_algo` |
| `compute_row_algo.m` | `pipeline/compute_row.m` | move + remove `_algo` |
| `store_row_algo.m` | `pipeline/store_row.m` | move + remove `_algo` |
| `cost_of.m` | `pipeline/cost_of.m` | move only |
| `vi_full_reference.m` | `reference/vi_full_reference.m` | move only |
| `compute_action_table_reference.m` | `reference/compute_action_table.m` | move + remove `_reference` |
| `coerce_transition_model.m` | `util/coerce_transition_model.m` | move only |
| `unpack_transitions.m` | `util/unpack_transitions.m` | move only |
| `make_goal_mask.m` | `util/make_goal_mask.m` | move only |
| `vi_params.m` | `vi_params.m` | no change |

## Function Name Updates

MATLAB requires file name = function name. For the 6 renamed files, update:

1. The `function` declaration line in the file itself
2. All call sites across the codebase

### Renamed Functions

| Old Function Name | New Function Name |
|-------------------|-------------------|
| `vi_sweep_stream_algo` | `vi_sweep_stream` |
| `stream_strip_algo` | `stream_strip` |
| `load_row_algo` | `load_row` |
| `compute_row_algo` | `compute_row` |
| `store_row_algo` | `store_row` |
| `compute_action_table_reference` | `compute_action_table` |

### Affected Callers

- `matlab/src/pipeline/vi_sweep_stream.m` — calls `stream_strip` (was `stream_strip_algo`)
- `matlab/src/pipeline/stream_strip.m` — calls `load_row`, `compute_row`, `store_row`
- `matlab/src/pipeline/compute_row.m` — calls `cost_of` (unchanged name)
- `matlab/src/reference/vi_full_reference.m` — calls `compute_action_table` (was `compute_action_table_reference`)
- `matlab/test/TestAlgorithmUnits.m` — calls pipeline functions
- `matlab/test/TestSolverIntegration.m` — calls `vi_sweep_stream`, `vi_full_reference`, `compute_action_table`
- `matlab/cosim/cosim_tb.m` — calls `vi_sweep_stream`, `compute_action_table`
- `matlab/model/create_model.m` — check for any src function references

## MATLAB Project Path

After file moves, the user must manually update the MATLAB project:
1. Open `value_iteration_fpga.prj` in MATLAB
2. Add `src/pipeline/`, `src/reference/`, `src/util/` to the project path
3. Remove stale file references from the project

## Validation

After refactoring, run `make matlab-sim` to confirm all tests pass with the new structure and names.
