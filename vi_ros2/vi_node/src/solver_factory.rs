//! Maps the `solver: string` ROS parameter to a `Box<dyn Solver>`.

use anyhow::{anyhow, Result};
use vi_algorithm::{
    BlockRefine, Frontier2D, Frontier3D, Frontier3DCoarseTheta, Frontier3DTau,
    Frontier3DTopK, FrontierStack, PyramidSweep, Reference, Solver,
};
use vi_core::params::MAX_OUTCOMES;

pub fn make_solver(name: &str) -> Result<Box<dyn Solver>> {
    Ok(match name {
        "reference" => Box::new(Reference { threshold: 0 }),
        "frontier3d" => Box::new(Frontier3D),
        "frontier3d_topk" => Box::new(Frontier3DTopK { k: MAX_OUTCOMES as u32 }),
        "frontier3d_tau" => Box::new(Frontier3DTau { tau: 0 }),
        "frontier3d_coarse_theta" => Box::new(Frontier3DCoarseTheta {
            coarse_step: 4,
            refine_iters: 200,
        }),
        "frontier2d" => Box::new(Frontier2D),
        "frontier_stack" => Box::new(FrontierStack),
        "block_refine" => Box::new(BlockRefine {
            block_w: 8,
            block_h: 8,
            local_sweeps: 2,
            threshold: 0,
        }),
        "pyramid" => Box::new(PyramidSweep {
            threshold: 0,
            min_size: 4,
            coarse_sweeps: 8,
            refine_sweeps: 50,
            descend_tau: 0,
        }),
        other => {
            return Err(anyhow!(
                "unknown solver: {other}. Supported: reference | frontier3d | frontier3d_topk | \
                 frontier3d_tau | frontier3d_coarse_theta | frontier2d | frontier_stack | \
                 block_refine | pyramid"
            ))
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_solvers_resolve_and_carry_name() {
        for name in [
            "reference",
            "frontier3d",
            "frontier3d_topk",
            "frontier3d_tau",
            "frontier3d_coarse_theta",
            "frontier2d",
            "frontier_stack",
            "block_refine",
            "pyramid",
        ] {
            let s = make_solver(name).expect(name);
            assert!(!s.name().is_empty(), "solver `{name}` returned empty name");
        }
    }

    #[test]
    fn unknown_solver_errors_with_listing() {
        let result = make_solver("does_not_exist");
        let err = match result {
            Err(e) => e.to_string(),
            Ok(_) => panic!("expected Err for unknown solver, got Ok"),
        };
        assert!(err.contains("does_not_exist"));
        assert!(err.contains("Supported"));
    }
}
