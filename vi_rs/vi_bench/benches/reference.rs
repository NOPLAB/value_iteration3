//! Criterion bench for the Reference solver.
//!
//! Tiny (size 8, MapType::Empty / Obstacle) matrix; the spec's CI gate lives in
//! `bench_summary --smoke`. The point here is just that `cargo bench` runs
//! without panicking.

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use vi_algorithm::context::{Budget, Solver};
use vi_algorithm::Reference;
use vi_bench::fixtures::build_context;
use vi_fixtures::{MapType, TransitionMode};

const SIZE: u32 = 8;
const BUDGET: Budget = Budget::Sweeps(50);

fn bench_reference(c: &mut Criterion) {
    let mut g = c.benchmark_group("reference");
    for (label, map_type) in [("empty", MapType::Empty), ("obstacle", MapType::Obstacle)] {
        let base = build_context(SIZE, SIZE, map_type, TransitionMode::Trivial);
        g.bench_with_input(BenchmarkId::new(label, SIZE), &base, |b, base| {
            b.iter_batched(
                || base.clone_value(),
                |mut ctx| {
                    let solver = Reference { threshold: 0 };
                    solver.run(&mut ctx, BUDGET);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
    g.finish();
}

criterion_group!(benches, bench_reference);
criterion_main!(benches);
