.PHONY: driver host test-host test-hw \
       csim hls vivado bitstream sync-hw-header \
       edf-docker edf-shell edf-setup edf-build \
       clean clean-fpga clean-edf

KERNEL ?=

# ---------- Software (driver + host) ----------

driver:
	$(MAKE) -C vi_fpga/driver/uio all

host: driver
	$(MAKE) -C vi_fpga/host all

test-host:
	$(MAKE) -C vi_fpga/host test-host

test-hw:
	$(MAKE) -C vi_fpga/host test-hw

# ---------- FPGA (HLS + Vivado) ----------
# Pass KERNEL= to select tile or stream, e.g.:
#   make csim KERNEL=stream
#   make bitstream KERNEL=tile

csim:
	$(MAKE) -C vi_fpga csim $(KERNEL)

hls:
	$(MAKE) -C vi_fpga hls $(KERNEL)

vivado:
	$(MAKE) -C vi_fpga vivado $(KERNEL)

bitstream:
	$(MAKE) -C vi_fpga bitstream $(KERNEL)

sync-hw-header:
	$(MAKE) -C vi_fpga/driver/uio sync-hw-header KERNEL=$(KERNEL)

# ---------- EDF / Linux (Docker) ----------

edf-docker:
	$(MAKE) -C vi_fpga/petalinux docker-build

edf-shell:
	$(MAKE) -C vi_fpga/petalinux docker-shell

edf-setup:
	$(MAKE) -C vi_fpga/petalinux edf-setup XSA=$(XSA)

edf-build:
	$(MAKE) -C vi_fpga/petalinux edf-build MACHINE=$(MACHINE)

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
	vivado -mode batch -source "vi_fpga/tcl/build_vivado.tcl" -tclargs matlab "vi_fpga/build"

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
	$(MAKE) -C vi_fpga/petalinux clean

clean-fpga:
	$(MAKE) -C vi_fpga clean

clean:
	$(MAKE) -C vi_fpga/driver/uio clean
	$(MAKE) -C vi_fpga/host clean

# ----- vi_ros2 (ROS2 Humble + ros2_rust) ------------------------------

VI_ROS2_DOCKER_IMG ?= vi_ros2_dev:humble
VI_COMPARE_ROS1_IMG ?= vi_compare_ros1:noetic

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

# ----- vi_compare (本家ROS1 vs vi_ros2 ROS2 ベンチ) -------------------

VI_ORIG ?= $(abspath $(PWD)/../value_iteration)

compare-build: ros2-docker
	docker build -t $(VI_COMPARE_ROS1_IMG) -f vi_compare/docker/Dockerfile.ros1 vi_compare/docker

compare-ros1:
	docker run --rm \
	  -v $(VI_ORIG):/src_value_iteration:ro \
	  -v $(PWD):/workspace -v $(PWD)/vi_compare/results:/results \
	  $(VI_COMPARE_ROS1_IMG) bash /workspace/vi_compare/ros1/run_ros1_bench.sh

compare-ros2:
	docker run --rm \
	  -v $(VI_ORIG):/src_value_iteration:ro \
	  -v $(PWD):/workspace -v $(PWD)/vi_compare/results:/results \
	  $(VI_ROS2_DOCKER_IMG) bash /workspace/vi_compare/ros2/run_ros2_bench.sh

compare-report:
	docker run --rm -v $(PWD):/workspace -v $(PWD)/vi_compare/results:/results \
	  $(VI_COMPARE_ROS1_IMG) bash -lc "cd /workspace/vi_compare/compare && python3 compare.py /results"

compare-bench: compare-build
	VI_ORIG=$(VI_ORIG) bash scripts/compare_bench.sh

.PHONY: compare-build compare-ros1 compare-ros2 compare-report compare-bench
