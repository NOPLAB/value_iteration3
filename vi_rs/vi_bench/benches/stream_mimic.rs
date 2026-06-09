//! Criterion bench for the u64 StreamMimic solver.
//!
//! Mirrors `block_pyramid.rs` shape — same (size=8) × (Empty, Obstacle)
//! matrix, sweep-based budget.

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use vi_bench::fixtures::{build_vi, BenchMap};
use vi_reference::solvers::{solve, U64Solver};

const SIZE: u32 = 8;
const MAX_SWEEPS: u32 = 200;

fn bench_stream_mimic(c: &mut Criterion) {
    let mut g = c.benchmark_group("stream_mimic");
    for (label, map) in [("empty", BenchMap::Empty), ("obstacle", BenchMap::Obstacle)] {
        g.bench_with_input(BenchmarkId::new(label, SIZE), &map, |b, &map| {
            b.iter_batched(
                || build_vi(SIZE, map),
                |mut vi| {
                    solve(&mut vi, U64Solver::StreamMimic, MAX_SWEEPS);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
    g.finish();
}

criterion_group!(benches, bench_stream_mimic);
criterion_main!(benches);
