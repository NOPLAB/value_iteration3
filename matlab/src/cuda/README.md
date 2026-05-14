# CUDA Staging Area

Use this directory for MATLAB-side CUDA experiments, such as:

- `parallel.gpu.CUDAKernel` prototypes
- GPU benchmark harnesses that mirror `workflows/benchmarks`
- MEX wrappers around native CUDA implementations

Keep shared MATLAB helpers in `src/common` or `src/shared`; keep this tree
focused on CUDA-specific entrypoints and adapters.
