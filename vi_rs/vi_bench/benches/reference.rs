//! Criterion bench for the u64 Reference solver (本家全走査の固定点).
//!
//! Tiny (size 8, Empty / Obstacle) matrix; the real CI gate lives in
//! `bench_summary --smoke`. The point here is just that `cargo bench` runs
//! without panicking.

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use vi_bench::fixtures::{build_vi, BenchMap};
use vi_reference::solvers::{solve, U64Solver};

const SIZE: u32 = 8;
const MAX_SWEEPS: u32 = 200;

fn bench_reference(c: &mut Criterion) {
    let mut g = c.benchmark_group("reference");
    for (label, map) in [("empty", BenchMap::Empty), ("obstacle", BenchMap::Obstacle)] {
        g.bench_with_input(BenchmarkId::new(label, SIZE), &map, |b, &map| {
            b.iter_batched(
                || build_vi(SIZE, map),
                |mut vi| {
                    solve(&mut vi, U64Solver::Reference, MAX_SWEEPS);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
    g.finish();
}

criterion_group!(benches, bench_reference);
criterion_main!(benches);
