//! Shared helpers for vi_bench benches and the bench_summary / bench_map
//! binaries.
//!
//! All solver benchmarking now targets the u64 (本家忠実) solvers in
//! `vi_reference::solvers`; `fixtures` builds a fully set-up `ValueIterator`
//! from a synthetic occupancy grid. `pgm` (PGM/YAML loading) is unchanged and
//! used by `bench_map`. `params` holds the reference action set / theta count
//! both of them measure against (previously `vi_core::params`).

pub mod fixtures;
pub mod params;
pub mod pgm;
