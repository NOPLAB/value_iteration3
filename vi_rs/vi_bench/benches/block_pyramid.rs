//! Criterion bench for the u64 BlockRefine and PyramidSweep solvers.

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use vi_bench::fixtures::{build_vi, BenchMap};
use vi_reference::solvers::{solve, U64Solver};

const SIZE: u32 = 8;
const MAX_SWEEPS: u32 = 200;

fn run_variant(c: &mut Criterion, group_name: &str, solver: U64Solver) {
    let mut g = c.benchmark_group(group_name);
    for (label, map) in [("empty", BenchMap::Empty), ("obstacle", BenchMap::Obstacle)] {
        g.bench_with_input(BenchmarkId::new(label, SIZE), &map, |b, &map| {
            b.iter_batched(
                || build_vi(SIZE, map),
                |mut vi| {
                    solve(&mut vi, solver, MAX_SWEEPS);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
    g.finish();
}

fn bench_block_pyramid(c: &mut Criterion) {
    run_variant(c, "block_refine", U64Solver::BlockRefine);
    run_variant(c, "pyramid_sweep", U64Solver::PyramidSweep);
}

criterion_group!(benches, bench_block_pyramid);
criterion_main!(benches);
