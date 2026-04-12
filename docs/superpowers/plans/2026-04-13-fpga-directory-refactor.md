# FPGA Directory Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Clean up the `fpga/` directory structure — shorten HLS directory names, consolidate TCL scripts, redirect build artifacts to `build/`.

**Architecture:** Pure file-move refactor with path updates. No code logic changes. HLS module names (`vi_sweep_stream`, `vi_sweep`) and hardware instance names are NOT renamed — they are baked into IP VLNV, bitstreams, and PYNQ overlays.

**Tech Stack:** GNU Make, Vitis HLS TCL, Vivado TCL, Git

---

### Before / After

```
BEFORE                                  AFTER
fpga/                                   fpga/
├── scripts/                            ├── tcl/
│   ├── export_hls_ip_stream.tcl        │   ├── export_hls_ip_stream.tcl
│   ├── export_hls_ip_tile.tcl          │   ├── export_hls_ip_tile.tcl
│   ├── run_csim_stream.tcl             │   ├── run_csim_stream.tcl
│   ├── run_csim_tile.tcl               │   ├── run_csim_tile.tcl
│   ├── build_vivado.tcl                │   ├── build_vivado.tcl
│   ├── hls_build_stream/ (artifact)    │   ├── create_project_stream.tcl  ← moved
│   └── hls_build_tile/   (artifact)    │   ├── create_project_tile.tcl    ← moved
├── hls/                                │   ├── create_bd_stream.tcl       ← moved
│   ├── vi_sweep_stream/                │   └── create_bd_tile.tcl         ← moved
│   └── vi_sweep_tile/                  ├── hls/
├── vivado/ultra96v2/                   │   ├── stream/        ← shortened
│   ├── create_project_stream.tcl       │   ├── tile/          ← shortened
│   ├── create_project_tile.tcl         │   └── study/
│   ├── create_bd_stream.tcl            ├── vivado/ultra96v2/
│   ├── create_bd_tile.tcl              │   ├── ip_repo_stream/
│   ├── ip_repo_stream/                 │   └── ip_repo_tile/
│   ├── ip_repo_tile/                   ├── pynq/
│   └── irq_notes.txt                   │   ├── stream/
├── pynq/                               │   └── tile/
│   ├── stream/                         ├── build/             ← new (gitignored)
│   └── tile/                           │   ├── hls_stream/    (artifact)
└── Makefile                            │   ├── hls_tile/      (artifact)
                                        │   ├── vi_stream/     (artifact)
                                        │   └── vi_tile/       (artifact)
                                        └── Makefile
```

### What does NOT change

- HLS top module names: `vi_sweep_stream` (stream), `vi_sweep` (tile)
- IP VLNV: `xilinx.com:hls:vi_sweep_stream:1.0`, `xilinx.com:hls:vi_sweep:1.0`
- BD instance names: `vi_sweep_stream_cu0/cu1`, `vi_sweep_cu0/cu1`
- PYNQ overlay attribute names: `self.ol.vi_sweep_stream_cu0` etc.
- HLS source filenames within `src/` and `tb/` (e.g. `vi_sweep_stream_top.cpp`)
- `hls_config.cfg` contents (they use relative paths within the HLS dir)

---

### Task 1: Rename HLS directories

Shorten `hls/vi_sweep_stream/` → `hls/stream/` and `hls/vi_sweep_tile/` → `hls/tile/`. The `vi_sweep_` prefix is redundant inside `hls/`.

**Files:**
- Move: `fpga/hls/vi_sweep_stream/` → `fpga/hls/stream/`
- Move: `fpga/hls/vi_sweep_tile/` → `fpga/hls/tile/`

- [ ] **Step 1: git mv the directories**

```bash
cd /c/Users/nop/dev/mywork/value_iteration_fpga
git mv fpga/hls/vi_sweep_stream fpga/hls/stream
git mv fpga/hls/vi_sweep_tile fpga/hls/tile
```

- [ ] **Step 2: Commit**

```bash
git add -A
git commit -m "refactor(fpga): rename hls/vi_sweep_{stream,tile} → hls/{stream,tile}"
```

---

### Task 2: Rename scripts/ → tcl/ and move Vivado TCL scripts

Consolidate all TCL scripts into one directory. Move the four Vivado TCL scripts from `vivado/ultra96v2/` into `tcl/`.

**Files:**
- Move: `fpga/scripts/` → `fpga/tcl/`
- Move: `fpga/vivado/ultra96v2/create_project_stream.tcl` → `fpga/tcl/`
- Move: `fpga/vivado/ultra96v2/create_project_tile.tcl` → `fpga/tcl/`
- Move: `fpga/vivado/ultra96v2/create_bd_stream.tcl` → `fpga/tcl/`
- Move: `fpga/vivado/ultra96v2/create_bd_tile.tcl` → `fpga/tcl/`

- [ ] **Step 1: git mv scripts → tcl**

```bash
git mv fpga/scripts fpga/tcl
```

- [ ] **Step 2: Move Vivado TCL scripts into tcl/**

```bash
git mv fpga/vivado/ultra96v2/create_project_stream.tcl fpga/tcl/
git mv fpga/vivado/ultra96v2/create_project_tile.tcl fpga/tcl/
git mv fpga/vivado/ultra96v2/create_bd_stream.tcl fpga/tcl/
git mv fpga/vivado/ultra96v2/create_bd_tile.tcl fpga/tcl/
```

- [ ] **Step 3: Commit**

```bash
git add -A
git commit -m "refactor(fpga): consolidate TCL scripts into fpga/tcl/"
```

---

### Task 3: Update TCL scripts for new paths

All TCL scripts use `$script_dir` to derive relative paths. Since both HLS dirs and the scripts themselves moved, every `$script_dir/../hls/vi_sweep_*` and `$project_dir` reference needs updating.

**Files:**
- Modify: `fpga/tcl/export_hls_ip_stream.tcl`
- Modify: `fpga/tcl/export_hls_ip_tile.tcl`
- Modify: `fpga/tcl/run_csim_stream.tcl`
- Modify: `fpga/tcl/run_csim_tile.tcl`
- Modify: `fpga/tcl/build_vivado.tcl`
- Modify: `fpga/tcl/create_project_stream.tcl`
- Modify: `fpga/tcl/create_project_tile.tcl`
- Modify: `fpga/tcl/create_bd_stream.tcl` (comment only)

- [ ] **Step 1: Update export_hls_ip_stream.tcl**

Change the `hls_dir` path (line 7) from `vi_sweep_stream` to `stream`:

```tcl
# OLD:
set hls_dir    [file normalize "$script_dir/../hls/vi_sweep_stream"]
# NEW:
set hls_dir    [file normalize "$script_dir/../hls/stream"]
```

The `ip_dst` path (line 8) stays the same — IP repo location is unchanged.

- [ ] **Step 2: Update export_hls_ip_tile.tcl**

Change the `hls_dir` path (line 7):

```tcl
# OLD:
set hls_dir    [file normalize "$script_dir/../hls/vi_sweep_tile"]
# NEW:
set hls_dir    [file normalize "$script_dir/../hls/tile"]
```

- [ ] **Step 3: Update run_csim_stream.tcl**

Change the `hls_dir` path (line 7):

```tcl
# OLD:
set hls_dir    [file normalize "$script_dir/../hls/vi_sweep_stream"]
# NEW:
set hls_dir    [file normalize "$script_dir/../hls/stream"]
```

- [ ] **Step 4: Update run_csim_tile.tcl**

Change the `hls_dir` path (line 7):

```tcl
# OLD:
set hls_dir    [file normalize "$script_dir/../hls/vi_sweep_tile"]
# NEW:
set hls_dir    [file normalize "$script_dir/../hls/tile"]
```

- [ ] **Step 5: Update build_vivado.tcl**

The `project_dir` (line 15) is fine — it still points to `vivado/ultra96v2`. But the `source` command (line 21) now needs to point to `tcl/` instead of `$project_dir/`:

```tcl
# OLD (line 21):
    source "$project_dir/create_project_${variant}.tcl"
# NEW:
    source "$script_dir/create_project_${variant}.tcl"
```

- [ ] **Step 6: Update create_project_stream.tcl**

This script was `source`d from build_vivado.tcl and used `[file dirname [info script]]` as project_dir. Now that it lives in `tcl/`, it needs an explicit path to `vivado/ultra96v2/`:

```tcl
# OLD (lines 5-7):
set project_name "vi_stream"
set project_dir  [file normalize [file dirname [info script]]]
set ip_repo_dir  [file normalize "$project_dir/ip_repo_stream"]

# NEW:
set project_name "vi_stream"
set project_dir  [file normalize "[file dirname [info script]]/../vivado/ultra96v2"]
set ip_repo_dir  [file normalize "$project_dir/ip_repo_stream"]
```

Also update the block design source path (line 19):

```tcl
# OLD:
source "$project_dir/create_bd_stream.tcl"
# NEW:
set tcl_dir [file normalize [file dirname [info script]]]
source "$tcl_dir/create_bd_stream.tcl"
```

- [ ] **Step 7: Update create_project_tile.tcl**

Same pattern as stream:

```tcl
# OLD (lines 5-7):
set project_name "vi_tile"
set project_dir  [file normalize [file dirname [info script]]]
set ip_repo_dir  [file normalize "$project_dir/ip_repo_tile"]

# NEW:
set project_name "vi_tile"
set project_dir  [file normalize "[file dirname [info script]]/../vivado/ultra96v2"]
set ip_repo_dir  [file normalize "$project_dir/ip_repo_tile"]
```

Also update the block design source path (line 19):

```tcl
# OLD:
source "$project_dir/create_bd_tile.tcl"
# NEW:
set tcl_dir [file normalize [file dirname [info script]]]
source "$tcl_dir/create_bd_tile.tcl"
```

- [ ] **Step 8: Commit**

```bash
git add fpga/tcl/
git commit -m "refactor(fpga): update TCL paths for new directory layout"
```

---

### Task 4: Redirect build artifacts to fpga/build/

HLS builds (`hls_build_stream/`, `hls_build_tile/`) currently land in `tcl/` (inherited from old `scripts/` cwd). Vivado projects (`vi_stream/`, `vi_tile/`) land in `vivado/ultra96v2/`. Redirect both to `fpga/build/`.

**Files:**
- Modify: `fpga/Makefile`
- Modify: `fpga/tcl/export_hls_ip_stream.tcl`
- Modify: `fpga/tcl/export_hls_ip_tile.tcl`
- Modify: `fpga/tcl/run_csim_stream.tcl`
- Modify: `fpga/tcl/run_csim_tile.tcl`
- Modify: `fpga/tcl/build_vivado.tcl`
- Modify: `fpga/tcl/create_project_stream.tcl`
- Modify: `fpga/tcl/create_project_tile.tcl`

- [ ] **Step 1: Update Makefile — add BUILD_DIR, change cwd for HLS commands**

The key change: HLS commands run from `BUILD_DIR` instead of `TCL_DIR`, so `open_project -reset hls_build_stream` creates artifacts there. Vivado gets `BUILD_DIR` passed as a tclarg.

```makefile
FPGA_DIR    := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
TCL_DIR     := $(FPGA_DIR)tcl/
VIVADO_DIR  := $(FPGA_DIR)vivado/ultra96v2/
BUILD_DIR   := $(FPGA_DIR)build/

# ---------- Kernel selector ----------
# Usage: make -C fpga csim stream
#        make -C fpga bitstream tile

ifneq ($(filter stream,$(MAKECMDGOALS)),)
  KERNEL := stream
endif
ifneq ($(filter tile,$(MAKECMDGOALS)),)
  KERNEL := tile
endif

stream tile:
	@:

.PHONY: stream tile csim hls vivado bitstream clean_hls clean_vivado clean

# ---------- Build targets ----------

csim:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && vitis-run --mode hls --tcl $(abspath $(TCL_DIR)run_csim_$(KERNEL).tcl)

hls:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && vitis-run --mode hls --tcl $(abspath $(TCL_DIR)export_hls_ip_$(KERNEL).tcl)
	cd $(VIVADO_DIR)ip_repo_$(KERNEL) && unzip -o export.zip

vivado: hls
	mkdir -p $(BUILD_DIR)
	vivado -mode batch -source $(TCL_DIR)build_vivado.tcl -tclargs $(KERNEL) $(abspath $(BUILD_DIR))

bitstream: vivado

# ---------- Clean ----------

clean_hls:
	rm -rf $(BUILD_DIR)hls_build_$(KERNEL)

clean_vivado:
	rm -rf $(BUILD_DIR)vi_$(KERNEL)

ifdef KERNEL
clean: clean_hls clean_vivado
else
clean:
	rm -rf $(BUILD_DIR)
endif

.PHONY: stream tile csim hls vivado bitstream clean_hls clean_vivado clean
```

Note: clean targets now use `rm -rf` (Unix). The old Windows `if exist ... rmdir` commands are replaced because the Makefile runs under bash (per CLAUDE.md the shell is bash on this Windows system, and Vitis tools run in a bash-like environment).

- [ ] **Step 2: Update build_vivado.tcl — accept build_dir arg, put project there**

```tcl
# OLD (lines 6-17):
if {$argc < 1} {
    error "Usage: vivado -mode batch -source build_vivado.tcl -tclargs <tile|stream>"
}
set variant [lindex $argv 0]
if {$variant ni {tile stream}} {
    error "Invalid variant '$variant'. Must be 'tile' or 'stream'."
}

set script_dir   [file normalize [file dirname [info script]]]
set project_dir  [file normalize "$script_dir/../vivado/ultra96v2"]
set project_name "vi_${variant}"
set xpr_file     "$project_dir/$project_name/$project_name.xpr"

# NEW:
if {$argc < 2} {
    error "Usage: vivado -mode batch -source build_vivado.tcl -tclargs <tile|stream> <build_dir>"
}
set variant   [lindex $argv 0]
set build_dir [file normalize [lindex $argv 1]]
if {$variant ni {tile stream}} {
    error "Invalid variant '$variant'. Must be 'tile' or 'stream'."
}

set script_dir   [file normalize [file dirname [info script]]]
set project_name "vi_${variant}"
set xpr_file     "$build_dir/$project_name/$project_name.xpr"
```

Also update the `source` for create_project (line 21) to pass `build_dir`:

```tcl
# OLD:
if {![file exists $xpr_file]} {
    puts "INFO: Project not found, creating..."
    source "$script_dir/create_project_${variant}.tcl"

# NEW:
if {![file exists $xpr_file]} {
    puts "INFO: Project not found, creating..."
    set ::build_dir $build_dir
    source "$script_dir/create_project_${variant}.tcl"
```

- [ ] **Step 3: Update create_project_stream.tcl — use build_dir for project location**

```tcl
# OLD (lines 5-10):
set project_name "vi_stream"
set project_dir  [file normalize "[file dirname [info script]]/../vivado/ultra96v2"]
set ip_repo_dir  [file normalize "$project_dir/ip_repo_stream"]
set part         "xczu3eg-sbva484-1-i"

create_project $project_name "$project_dir/$project_name" -part $part -force

# NEW:
set project_name "vi_stream"
set tcl_dir      [file normalize [file dirname [info script]]]
set vivado_dir   [file normalize "$tcl_dir/../vivado/ultra96v2"]
set ip_repo_dir  [file normalize "$vivado_dir/ip_repo_stream"]
set part         "xczu3eg-sbva484-1-i"

create_project $project_name "$::build_dir/$project_name" -part $part -force
```

Update the glob for HDL wrapper (line 26) and puts (line 29):

```tcl
# OLD:
add_files -norecurse [glob "$project_dir/$project_name/$project_name.gen/sources_1/bd/vi_bd/hdl/vi_bd_wrapper.v"]
...
puts "INFO: Project created at $project_dir/$project_name"

# NEW:
add_files -norecurse [glob "$::build_dir/$project_name/$project_name.gen/sources_1/bd/vi_bd/hdl/vi_bd_wrapper.v"]
...
puts "INFO: Project created at $::build_dir/$project_name"
```

- [ ] **Step 4: Update create_project_tile.tcl — same pattern as stream**

```tcl
# OLD (lines 5-10):
set project_name "vi_tile"
set project_dir  [file normalize "[file dirname [info script]]/../vivado/ultra96v2"]
set ip_repo_dir  [file normalize "$project_dir/ip_repo_tile"]
set part         "xczu3eg-sbva484-1-i"

create_project $project_name "$project_dir/$project_name" -part $part -force

# NEW:
set project_name "vi_tile"
set tcl_dir      [file normalize [file dirname [info script]]]
set vivado_dir   [file normalize "$tcl_dir/../vivado/ultra96v2"]
set ip_repo_dir  [file normalize "$vivado_dir/ip_repo_tile"]
set part         "xczu3eg-sbva484-1-i"

create_project $project_name "$::build_dir/$project_name" -part $part -force
```

Update glob + puts same as stream:

```tcl
add_files -norecurse [glob "$::build_dir/$project_name/$project_name.gen/sources_1/bd/vi_bd/hdl/vi_bd_wrapper.v"]
...
puts "INFO: Project created at $::build_dir/$project_name"
```

- [ ] **Step 5: Update HLS export TCL scripts — change open_project name**

In `export_hls_ip_stream.tcl`, the project name `hls_build_stream` is created in cwd (now `build/`). Keep the name:

```tcl
# No change needed to open_project line — it's cwd-relative and cwd is now build/
```

Only the `hls_dir` path changes (already done in Task 3). No additional changes needed here.

Same for `export_hls_ip_tile.tcl`, `run_csim_stream.tcl`, `run_csim_tile.tcl`.

- [ ] **Step 6: Commit**

```bash
git add fpga/Makefile fpga/tcl/
git commit -m "refactor(fpga): redirect build artifacts to fpga/build/"
```

---

### Task 5: Update .gitignore

Replace old paths with new patterns.

**Files:**
- Modify: `.gitignore`

- [ ] **Step 1: Update .gitignore**

```gitignore
# OLD HLS patterns:
fpga/**/solution*/
fpga/**/hls_build*/

# NEW — build artifacts now in fpga/build/:
fpga/build/

# OLD Vivado project patterns:
fpga/vivado/ultra96v2/vi_tile/
fpga/vivado/ultra96v2/vi_stream/
fpga/vivado/ultra96v2/vi_ultra96v2/
fpga/vivado/ultra96v2/ip_repo_tile/
fpga/vivado/ultra96v2/ip_repo_stream/
fpga/vivado/ultra96v2/ip_repo/

# NEW — keep ip_repo ignores (they're generated), drop project dirs (now in build/):
fpga/vivado/ultra96v2/ip_repo*/
```

Full replacement for the fpga-related sections:

```gitignore
# FPGA build artifacts
fpga/build/
fpga/**/solution*/

# Vivado generated IP repos
fpga/vivado/ultra96v2/ip_repo*/
```

- [ ] **Step 2: Commit**

```bash
git add .gitignore
git commit -m "chore: update .gitignore for new fpga directory layout"
```

---

### Task 6: Update CLAUDE.md and PYNQ docstrings

Update documentation references to the old paths.

**Files:**
- Modify: `CLAUDE.md`
- Modify: `fpga/pynq/stream/vi_overlay_stream.py` (docstring only)
- Modify: `fpga/pynq/tile/vi_overlay_tile.py` (docstring only)

- [ ] **Step 1: Update CLAUDE.md FPGA build section**

Replace the FPGA build section (lines 22-30) with:

```markdown
### FPGA build (`fpga/Makefile`)

Tools must be on `PATH` — invoke bare `vitis-run` / `vivado` (Vitis 2025.2). Do **not** prefix with `source settings.sh`. Tile and streaming kernels have fully separate build paths. All TCL scripts live in `fpga/tcl/`; build artifacts go to `fpga/build/`.

- `make -C fpga csim tile` — HLS C-simulation of tile-based kernel (`fpga/hls/tile/`).
- `make -C fpga csim stream` — HLS C-simulation of streaming kernel (`fpga/hls/stream/`).
- `make -C fpga hls tile` — HLS synth + IP export (tile) into `fpga/build/hls_build_tile/`, IP to `ip_repo_tile/`.
- `make -C fpga hls stream` — HLS synth + IP export (streaming) into `fpga/build/hls_build_stream/`, IP to `ip_repo_stream/`.
- `make -C fpga bitstream tile` — HLS + Vivado synthesis + bitstream for tile kernel, project `fpga/build/vi_tile/`.
- `make -C fpga bitstream stream` — HLS + Vivado synthesis + bitstream for streaming kernel, project `fpga/build/vi_stream/`.
- `make -C fpga clean` — clean both tile and stream build artifacts. Append `tile` or `stream` to clean one.
```

- [ ] **Step 2: Update CLAUDE.md Architecture section**

Replace path references (lines 34-37):

```markdown
Four vertically integrated layers share the same 16-bit data contract defined in `fpga/hls/tile/src/vi_types.h` (tile-based) and `fpga/hls/stream/src/vi_stream_types.h` (streaming). Keep them in sync.

### 1. HLS kernel (`fpga/hls/tile/` and `fpga/hls/stream/`)
```

- [ ] **Step 3: Update vi_overlay_stream.py docstring**

Change the register offset path in the docstring (line 10):

```python
# OLD:
#   ip_repo_stream/drivers/vi_sweep_stream_v1_0/src/xvi_sweep_stream_hw.h
# NEW:
#   vivado/ultra96v2/ip_repo_stream/drivers/vi_sweep_stream_v1_0/src/xvi_sweep_stream_hw.h
```

- [ ] **Step 4: Update vi_overlay_tile.py docstring**

Change the register offset path in the docstring:

```python
# OLD:
#   hls_build_tile/solution1/impl/misc/drivers/vi_sweep_v1_0/src/xvi_sweep_hw.h
# NEW:
#   vivado/ultra96v2/ip_repo_tile/drivers/vi_sweep_v1_0/src/xvi_sweep_hw.h
```

- [ ] **Step 5: Commit**

```bash
git add CLAUDE.md fpga/pynq/stream/vi_overlay_stream.py fpga/pynq/tile/vi_overlay_tile.py
git commit -m "docs: update paths in CLAUDE.md and PYNQ docstrings for fpga refactor"
```

---

### Task 7: Clean up stray Vivado files

Remove `irq_notes.txt` and any stray files from `vivado/ultra96v2/` now that TCL scripts have moved out. Verify the directory only contains `ip_repo_*/`.

**Files:**
- Check: `fpga/vivado/ultra96v2/` — after TCL moves, only `ip_repo_stream/` and `ip_repo_tile/` should remain (both gitignored). `irq_notes.txt` can move to `fpga/` or be deleted if obsolete.

- [ ] **Step 1: Check what remains in vivado/ultra96v2/**

```bash
ls fpga/vivado/ultra96v2/
```

If only gitignored dirs + `irq_notes.txt` remain, decide: keep `irq_notes.txt` in place or move it.

- [ ] **Step 2: Commit any cleanup**

```bash
git add -A
git commit -m "chore: clean up stray files after fpga directory refactor"
```
