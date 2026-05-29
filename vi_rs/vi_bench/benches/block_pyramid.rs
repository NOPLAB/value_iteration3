//! Criterion bench for BlockRefine and PyramidSweep.

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use vi_algorithm::context::{Budget, Solver};
use vi_algorithm::{BlockRefine, PyramidSweep};
use vi_bench::fixtures::build_context;
use vi_fixtures::{MapType, TransitionMode};

const SIZE: u32 = 8;
const BUDGET: Budget = Budget::Sweeps(50);

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

fn bench_block_pyramid(c: &mut Criterion) {
    run_variant(c, "block_refine", || BlockRefine {
        block_w: 8,
        block_h: 8,
        local_sweeps: 2,
        threshold: 0,
    });
    run_variant(c, "pyramid_sweep", || PyramidSweep {
        threshold: 0,
        min_size: 4,
        coarse_sweeps: 8,
        refine_sweeps: 50,
        // descend_tau=0 to time the *exact* configuration (matches bench_summary
        // and the parity tests); a nonzero tau would benchmark a faster but
        // non-bit-exact variant.
        descend_tau: 0,
    });
}

criterion_group!(benches, bench_block_pyramid);
criterion_main!(benches);
