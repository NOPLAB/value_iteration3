//! Criterion bench for internal bitboard ops.
//!
//! Tiny boards (8×8×60 for the 3-D dilate, 16×16 for the 2-D AND/OR) — the
//! goal is to confirm the bench harness wires up, not to produce production
//! throughput numbers.

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use vi_algorithm::bitboard::{Bitboard2D, Bitboard3D};

const N_THETA: u32 = 60;
const BB3_X: u32 = 8;
const BB3_Y: u32 = 8;
const BB2_X: u32 = 16;
const BB2_Y: u32 = 16;

fn bench_bitboard(c: &mut Criterion) {
    // --- Bitboard3D::dilate(1, 1, 1) on 8×8×60 with a single seeded bit. ---
    let mut bb3 = Bitboard3D::new(BB3_X, BB3_Y, N_THETA);
    bb3.set(BB3_X / 2, BB3_Y / 2, N_THETA / 2);
    c.bench_function("bitboard3d_dilate_1_1_1", |b| {
        b.iter(|| {
            let out = black_box(&bb3).dilate(1, 1, 1);
            black_box(out);
        });
    });

    // --- Bitboard2D::or_inplace on 16×16. ---
    let mut a_or = Bitboard2D::new(BB2_X, BB2_Y);
    let mut b_or = Bitboard2D::new(BB2_X, BB2_Y);
    // Seed two distinct patterns so OR has real work to do.
    for i in 0..BB2_X {
        a_or.set(i, i % BB2_Y);
        b_or.set((i + 1) % BB2_X, i % BB2_Y);
    }
    c.bench_function("bitboard2d_or_inplace_16x16", |b| {
        b.iter_batched(
            || a_or.clone(),
            |mut acc| {
                acc.or_inplace(black_box(&b_or));
                black_box(acc);
            },
            criterion::BatchSize::SmallInput,
        );
    });

    // --- Bitboard2D::and_inplace on 16×16. ---
    let mut a_and = Bitboard2D::new(BB2_X, BB2_Y);
    let mut b_and = Bitboard2D::new(BB2_X, BB2_Y);
    for iy in 0..BB2_Y {
        for ix in 0..BB2_X {
            // Overlapping but distinct patterns so AND output is non-trivial.
            if (ix + iy) % 2 == 0 {
                a_and.set(ix, iy);
            }
            if (ix * iy) % 3 != 0 {
                b_and.set(ix, iy);
            }
        }
    }
    c.bench_function("bitboard2d_and_inplace_16x16", |b| {
        b.iter_batched(
            || a_and.clone(),
            |mut acc| {
                acc.and_inplace(black_box(&b_and));
                black_box(acc);
            },
            criterion::BatchSize::SmallInput,
        );
    });
}

criterion_group!(benches, bench_bitboard);
criterion_main!(benches);
