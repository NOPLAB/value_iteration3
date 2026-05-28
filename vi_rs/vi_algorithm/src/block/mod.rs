//! Block-based and multigrid value-iteration solvers.
//!
//! Mirrors `vi_matlab/src/cpu/block/`.
//! See `docs/superpowers/specs/2026-05-22-vi-rs-algorithm-port-design.md` §4.2.

pub mod pyramid;
pub mod refine;

pub use pyramid::PyramidSweep;
pub use refine::BlockRefine;
