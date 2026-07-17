//! Maps the `solver: string` ROS parameter to a `vi_reference::solvers::U64Solver`.
//!
//! u16 時代は `Box<dyn Solver>` を返していたが、u64 (本家忠実) 移行で `U64Solver`
//! (Copy な enum) を返す。名前解決は `U64Solver::from_name` へ委譲する —
//! ここに独自ホワイトリストを持つと本体に追加されたソルバ (frontier2d_sparse 等)
//! が ROS 側から選べなくなるため。近似ソルバは from_name が no-op パラメータ
//! (tau=0 / k=全 outcome / step=1) = Frontier3D 等価で返す (本家と bit-exact)。
//!
//! 旧 ROS パラメータ名 "pyramid" のみ互換エイリアスとしてここで吸収する。

use anyhow::{anyhow, Result};
use vi_reference::solvers::U64Solver;

pub fn make_solver(name: &str) -> Result<U64Solver> {
    // 旧 ROS パラメータ名 "pyramid" を PyramidSweep にマップ ("pyramid_sweep" も可)。
    let canonical = if name == "pyramid" { "pyramid_sweep" } else { name };
    U64Solver::from_name(canonical).ok_or_else(|| {
        anyhow!(
            "unknown solver: {name}. Supported: reference | frontier3d | frontier3d_topk | \
             frontier3d_tau | frontier3d_coarse_theta | frontier2d | frontier2d_soa | \
             frontier2d_pad | frontier2d_par | frontier2d_par_unsafe | frontier2d_fused | \
             frontier2d_sparse | frontier2d_sparse_compact | frontier_stack | block_refine | \
             pyramid | pyramid_sweep | stream_mimic | prio_ls | prio_lc"
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_solvers_resolve() {
        for name in [
            "reference",
            "frontier3d",
            "frontier3d_topk",
            "frontier3d_tau",
            "frontier3d_coarse_theta",
            "frontier2d",
            "frontier2d_sparse",
            "frontier2d_sparse_compact",
            "frontier_stack",
            "block_refine",
            "pyramid",
            "pyramid_sweep",
            "stream_mimic",
        ] {
            make_solver(name).unwrap_or_else(|_| panic!("solver `{name}` must resolve"));
        }
    }

    #[test]
    fn pyramid_alias_maps_to_pyramid_sweep() {
        assert_eq!(make_solver("pyramid").unwrap(), U64Solver::PyramidSweep);
        assert_eq!(make_solver("pyramid_sweep").unwrap(), U64Solver::PyramidSweep);
    }

    #[test]
    fn sparse_solver_resolves_for_planner_use() {
        assert_eq!(
            make_solver("frontier2d_sparse").unwrap(),
            U64Solver::Frontier2DSparse
        );
    }

    #[test]
    fn unknown_solver_errors_with_listing() {
        let err = match make_solver("does_not_exist") {
            Err(e) => e.to_string(),
            Ok(_) => panic!("expected Err for unknown solver"),
        };
        assert!(err.contains("does_not_exist"));
        assert!(err.contains("Supported"));
    }
}
