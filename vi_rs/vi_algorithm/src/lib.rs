//! Value-iteration algorithms with a uniform `Solver` trait.
//!
//! See `docs/superpowers/specs/2026-05-22-vi-rs-algorithm-port-design.md` §4.

pub mod bitboard;
pub mod context;
pub mod kernel;
pub mod reference;

pub use context::{Budget, MapDims, SolveExtra, SolveStats, Solver, VIContext};
pub use reference::Reference;
