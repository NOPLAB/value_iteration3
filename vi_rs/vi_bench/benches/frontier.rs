//! Criterion bench for all six frontier solver variants.
//!
//! Tiny (size 8, Empty / Obstacle) matrix per variant. Iterations(500) gives
//! every frontier solver room to terminate.

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use vi_algorithm::context::{Budget, Solver};
use vi_algorithm::{
    Frontier2D, Frontier3D, Frontier3DCoarseTheta, Frontier3DTau, Frontier3DTopK, FrontierStack,
};
use vi_bench::fixtures::build_context;
use vi_core::params::MAX_OUTCOMES;
use vi_fixtures::{MapType, TransitionMode};

const SIZE: u32 = 8;
const BUDGET: Budget = Budget::Iterations(500);
// k >= MAX_OUTCOMES → no pruning (Frontier3DTopK reduces to plain Frontier3D).
const TOPK_NO_PRUNE: u32 = MAX_OUTCOMES as u32;

fn run_variant<S: Solver>(c: &mut Criterion, group_name: &str, make_solver: impl Fn() -> S) {
    let mut g = c.benchmark_group(group_name);
    for (label, map_type) in [("empty", MapType::Empty), ("obstacle", MapType::Obstacle)] {
        let base = build_context(SIZE, SIZE, map_type, TransitionMode::Trivial);
        g.bench_with_input(BenchmarkId::new(label, SIZE), &base, |b, base| {
            b.iter_batched(
                || base.clone_value(),
                |mut ctx| {
                    let solver = make_solver();
                    solver.run(&mut ctx, BUDGET);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
    g.finish();
}

fn bench_frontier(c: &mut Criterion) {
    run_variant(c, "frontier_2d", || Frontier2D);
    run_variant(c, "frontier_3d", || Frontier3D);
    run_variant(c, "frontier_stack", || FrontierStack);
    run_variant(c, "frontier_3d_tau", || Frontier3DTau { tau: 0 });
    run_variant(c, "frontier_3d_topk", || Frontier3DTopK { k: TOPK_NO_PRUNE });
    run_variant(c, "frontier_3d_coarse_theta", || Frontier3DCoarseTheta {
        coarse_step: 4,
        refine_iters: 200,
    });
}

criterion_group!(benches, bench_frontier);
criterion_main!(benches);
