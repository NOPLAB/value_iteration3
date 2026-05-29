//! Criterion bench for StreamMimic.
//!
//! Mirrors `block_pyramid.rs` shape — same (size=8) × (Empty, Obstacle)
//! matrix, sweep-based budget.

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use vi_algorithm::context::{Budget, Solver};
use vi_algorithm::StreamMimic;
use vi_bench::fixtures::build_context;
use vi_fixtures::{MapType, TransitionMode};

const SIZE: u32 = 8;
const BUDGET: Budget = Budget::Sweeps(50);

fn bench_stream_mimic(c: &mut Criterion) {
    let mut g = c.benchmark_group("stream_mimic");
    for (label, map_type) in [("empty", MapType::Empty), ("obstacle", MapType::Obstacle)] {
        let base = build_context(SIZE, SIZE, map_type, TransitionMode::Trivial);
        g.bench_with_input(BenchmarkId::new(label, SIZE), &base, |b, base| {
            b.iter_batched(
                || base.clone_value(),
                |mut ctx| {
                    let solver = StreamMimic { threshold: 0 };
                    solver.run(&mut ctx, BUDGET);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
    g.finish();
}

criterion_group!(benches, bench_stream_mimic);
criterion_main!(benches);
