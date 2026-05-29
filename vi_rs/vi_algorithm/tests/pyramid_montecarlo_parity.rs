//! Bit-exact parity of PyramidSweep vs Reference under the *probabilistic*
//! (PaperMonteCarlo) transition model, per spec §4.8 / §7.2.
//!
//! The deterministic helpers in `frontier::test_helpers` use single-outcome
//! transitions, whose floored Bellman fixed point is unique. The probabilistic
//! model has multi-outcome averaging + integer floor, whose fixed point is NOT
//! unique (±1, seed-dependent). PyramidSweep must converge to the SAME fixed
//! point Reference does — i.e. from above — to stay an exact oracle. This test
//! locks that in on maps large enough to exercise the pyramid (≥ 2 levels).

use vi_algorithm::context::{Budget, MapDims, Solver, VIContext};
use vi_algorithm::{PyramidSweep, Reference};
use vi_fixtures::{generate_map, generate_transitions, MapType, TransitionMode};

fn montecarlo_ctx(size: u32, map_type: MapType) -> VIContext {
    let m = generate_map(size, size, map_type);
    let transitions =
        generate_transitions(TransitionMode::PaperMonteCarlo { xy_resolution: 0.05 }).unpack();
    VIContext {
        dims: MapDims { map_x: size, map_y: size },
        value: m.value,
        penalty: m.penalty,
        goal_mask: m.goal_mask,
        transitions,
    }
}

#[test]
fn pyramid_bit_exact_vs_reference_montecarlo() {
    let cases: &[(MapType, &str)] = &[
        (MapType::Empty, "empty"),
        (MapType::Obstacle, "obstacle"),
        (MapType::Sentinel, "sentinel"),
    ];
    for &size in &[16u32, 32] {
        for &(map_type, label) in cases {
            let base = montecarlo_ctx(size, map_type);

            let mut rc = base.clone_value();
            let rs = Reference { threshold: 0 }.run(&mut rc, Budget::Sweeps(200));
            assert!(rs.converged, "Reference must converge: size={size} {label}");

            let mut pc = base.clone_value();
            PyramidSweep {
                threshold: 0,
                min_size: 4,
                coarse_sweeps: 8,
                refine_sweeps: 50,
                descend_tau: 0,
            }
            .run(&mut pc, Budget::Sweeps(200));

            let mismatch = rc
                .value
                .iter()
                .zip(pc.value.iter())
                .filter(|(a, b)| a != b)
                .count();
            assert_eq!(
                mismatch, 0,
                "PyramidSweep must be bit-exact with Reference: size={size} {label}, mismatch={mismatch}"
            );
        }
    }
}
