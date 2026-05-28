//! Fixture builders shared by benches and the `bench_summary` binary.

use vi_algorithm::context::{MapDims, VIContext};
use vi_fixtures::{generate_map, generate_transitions, MapType, TransitionMode};

/// Build a fully-populated [`VIContext`] from a fixture spec. Used by benches
/// and the `bench_summary` CLI so both consume identical inputs.
pub fn build_context(
    map_x: u32,
    map_y: u32,
    map_type: MapType,
    trans_mode: TransitionMode,
) -> VIContext {
    let m = generate_map(map_x, map_y, map_type);
    let packed = generate_transitions(trans_mode);
    let transitions = packed.unpack();
    VIContext {
        dims: MapDims { map_x, map_y },
        value: m.value,
        penalty: m.penalty,
        goal_mask: m.goal_mask,
        transitions,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_context_has_requested_dimensions() {
        let ctx = build_context(8, 6, MapType::Empty, TransitionMode::Trivial);
        assert_eq!(ctx.dims.map_x, 8);
        assert_eq!(ctx.dims.map_y, 6);
        // ndarray shape is (my, mx, n_theta).
        assert_eq!(ctx.value.shape()[0], 6);
        assert_eq!(ctx.value.shape()[1], 8);
        assert_eq!(ctx.penalty.shape(), &[6, 8]);
        assert_eq!(ctx.goal_mask.shape()[0], 6);
        assert_eq!(ctx.goal_mask.shape()[1], 8);
    }
}
