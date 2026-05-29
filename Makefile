.PHONY: driver host test-host test-hw \
       csim hls vivado bitstream sync-hw-header \
       edf-docker edf-shell edf-setup edf-build \
       clean clean-fpga clean-edf

KERNEL ?=

# ---------- Software (driver + host) ----------

driver:
	$(MAKE) -C driver/uio all

host: driver
	$(MAKE) -C host all

test-host:
	$(MAKE) -C host test-host

test-hw:
	$(MAKE) -C host test-hw

# ---------- FPGA (HLS + Vivado) ----------
# Pass KERNEL= to select tile or stream, e.g.:
#   make csim KERNEL=stream
#   make bitstream KERNEL=tile

csim:
	$(MAKE) -C fpga csim $(KERNEL)

hls:
	$(MAKE) -C fpga hls $(KERNEL)

vivado:
	$(MAKE) -C fpga vivado $(KERNEL)

bitstream:
	$(MAKE) -C fpga bitstream $(KERNEL)

sync-hw-header:
	$(MAKE) -C driver/uio sync-hw-header KERNEL=$(KERNEL)

# ---------- EDF / Linux (Docker) ----------

edf-docker:
	$(MAKE) -C petalinux docker-build

edf-shell:
	$(MAKE) -C petalinux docker-shell

edf-setup:
	$(MAKE) -C petalinux edf-setup XSA=$(XSA)

edf-build:
	$(MAKE) -C petalinux edf-build MACHINE=$(MACHINE)

# ---------- MATLAB (HDL Coder) ----------

.PHONY: matlab-sim matlab-hdl matlab-cosim matlab-bitstream \
        matlab-bench matlab-bench-codegen matlab-codegen-build matlab-codegen-clean

matlab-sim:
	cd vi_matlab && matlab -batch "run_matlab_tests"

matlab-hdl:
	cd vi_matlab && matlab -batch "setup_matlab_paths('fpga-export'); export_repo_ip"

matlab-cosim:
	cd vi_matlab && matlab -batch "setup_matlab_paths('validation','tests'); cosim_tb"

matlab-bitstream: matlab-hdl
	vivado -mode batch -source "fpga/tcl/build_vivado.tcl" -tclargs matlab "fpga/build"

matlab-bench:
	cd vi_matlab && matlab -batch "setup_matlab_paths('src','tests','bench'); benchmark_vi"

# MATLAB Coder C-generation benchmark: builds MEX from vi_full_reference and
# vi_sweep_stream_algo, then compares MATLAB vs codegen-C timings on bench_cases.
# Pass REBUILD=1 to force a clean rebuild of the MEX artifacts.
REBUILD ?= 0
matlab-bench-codegen:
	cd vi_matlab && matlab -batch "setup_matlab_paths('src','tests','bench'); benchmark_vi_codegen('rebuild', logical($(REBUILD)))"

matlab-codegen-build:
	cd vi_matlab && matlab -batch "setup_matlab_paths('src','tests','bench'); codegen_build('rebuild', logical($(REBUILD)))"

matlab-codegen-clean:
	rm -rf vi_matlab/artifacts/benchmarks/codegen_build

# ---------- Rust (vi_rs workspace) ----------

.PHONY: rs-test rs-bench rs-bench-summary rs-bench-parallel

rs-test:
	cd vi_rs && cargo test --workspace

rs-bench:
	cd vi_rs && cargo bench -p vi_bench

rs-bench-summary:
	cd vi_rs && cargo run --release -p vi_bench --bin bench_summary -- \
	    --sizes 8,16,32,64 --types empty,obstacle,sentinel,random \
	    --markdown --out target/bench_results/summary_$(shell date +%Y%m%d_%H%M%S).csv

rs-bench-parallel:
	cd vi_rs && cargo run --release -p vi_bench --features parallel \
	    --bin bench_summary -- --parallel --markdown \
	    --out target/bench_results/summary_parallel_$(shell date +%Y%m%d_%H%M%S).csv

# ---------- Clean ----------

clean-edf:
	$(MAKE) -C petalinux clean

clean-fpga:
	$(MAKE) -C fpga clean

clean:
	$(MAKE) -C driver/uio clean
	$(MAKE) -C host clean

# ----- vi_ros2 (ROS2 Humble + ros2_rust) ------------------------------

VI_ROS2_DOCKER_IMG ?= vi_ros2_dev:humble

ros2-docker:
	docker build -t $(VI_ROS2_DOCKER_IMG) vi_ros2/docker

ros2-shell:
	docker run --rm -it \
	  -v $(PWD):/workspace \
	  -w /workspace \
	  $(VI_ROS2_DOCKER_IMG)

ros2-build:
	docker run --rm \
	  -v $(PWD):/workspace \
	  -w /workspace \
	  $(VI_ROS2_DOCKER_IMG) \
	  bash scripts/ros2_build.sh

ros2-test:
	docker run --rm \
	  -v $(PWD):/workspace \
	  -w /workspace \
	  $(VI_ROS2_DOCKER_IMG) \
	  bash scripts/ros2_test.sh

.PHONY: ros2-docker ros2-shell ros2-build ros2-test
