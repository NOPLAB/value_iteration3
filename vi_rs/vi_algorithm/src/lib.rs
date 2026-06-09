//! Word-parallel bitboard primitives for value-iteration frontier solvers.
//!
//! The u16 `Solver` family (reference / frontier / block / pyramid / stream)
//! that used to live here has been ported to the 本家忠実 u64 model in
//! `vi_reference::solvers` and removed. What remains is the value-type-agnostic
//! `bitboard` module (3-D θ-periodic dilation, 2-D AND/OR, enumerate, ndarray
//! conv), which `vi_reference`'s frontier solvers and the `vi_bench` bitboard
//! microbench still depend on.
//!
//! See `docs/superpowers/specs/2026-05-22-vi-rs-algorithm-port-design.md` §4 and
//! `docs/superpowers/specs/2026-06-09-vi-u64-fast-solvers-design.md`.

pub mod bitboard;
