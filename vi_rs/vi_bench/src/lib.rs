//! Shared helpers for vi_bench benches and the bench_summary binary.
//!
//! See `docs/superpowers/specs/2026-05-22-vi-rs-algorithm-port-design.md` §6.
//!
//! Note: spec §6.2 lists `stream_mimic.rs` as a bench file; that bench is
//! omitted because `StreamMimic` is not yet implemented in `vi_algorithm`.
//! It will be added once the StreamMimic solver lands.

pub mod fixtures;
pub mod pgm;
