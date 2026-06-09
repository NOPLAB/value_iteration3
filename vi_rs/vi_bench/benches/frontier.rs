//! Criterion bench for the u64 frontier-family solvers.
//!
//! Tiny (size 8, Empty / Obstacle) matrix per variant. `max_iters=4000` gives
//! every frontier solver room to terminate. 近似ソルバは no-op パラメータ
//! （tau=0 / k=全 outcome / step=1）で計測する（= Frontier3D 等価）。

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use vi_bench::fixtures::{build_vi, BenchMap};
use vi_reference::solvers::{solve, U64Solver};

const SIZE: u32 = 8;
const MAX_ITERS: u32 = 4000;

fn run_variant(c: &mut Criterion, group_name: &str, solver: U64Solver) {
    let mut g = c.benchmark_group(group_name);
    for (label, map) in [("empty", BenchMap::Empty), ("obstacle", BenchMap::Obstacle)] {
        g.bench_with_input(BenchmarkId::new(label, SIZE), &map, |b, &map| {
            b.iter_batched(
                || build_vi(SIZE, map),
                |mut vi| {
                    solve(&mut vi, solver, MAX_ITERS);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
    g.finish();
}

fn bench_frontier(c: &mut Criterion) {
    run_variant(c, "frontier_2d", U64Solver::Frontier2D);
    run_variant(c, "frontier_3d", U64Solver::Frontier3D);
    run_variant(c, "frontier_stack", U64Solver::FrontierStack);
    run_variant(c, "frontier_3d_tau", U64Solver::Frontier3DTau { tau: 0 });
    run_variant(c, "frontier_3d_topk", U64Solver::Frontier3DTopK { k: u32::MAX });
    run_variant(c, "frontier_3d_coarse_theta", U64Solver::Frontier3DCoarseTheta { step: 1 });
}

criterion_group!(benches, bench_frontier);
criterion_main!(benches);
