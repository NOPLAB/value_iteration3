//! Invalidation across worker modes and compact recentering.
use super::invalidation::{drain, policies};
use super::*;

#[test]
fn pending_invalidation_pauses_guidance_but_preserves_terminal_decisions() {
    for compact in [false, true] {
        let mut core = managed_field(compact);
        finish_in_background(&mut core, 100_000);
        core.cfg.scan_penalty = 20_000;
        core.observe_scan(
            &LaserScan {
                angle_min: 0.0,
                angle_increment: 0.0,
                ranges: vec![0.2],
            },
            pose(1.6, 1.6, 0.0),
        );
        let robot = pose(1.6, 1.6, 0.0);
        let goal = pose(2.8, 1.6, 0.0);
        for kind in [FollowKind::Greedy, FollowKind::Dwa, FollowKind::Mppi] {
            core.cfg.follow_controller = kind;
            core.follow = follow::make_controller(&core.cfg, &core.build.actions);
            core.set_window(robot);
            assert_eq!(core.decide(robot), Decision::NoAction);
            core.set_window(goal);
            assert_eq!(core.decide(goal), Decision::Goal);
        }
        core.set_window(robot);
        assert_eq!(core.decide_qmdp(&[(robot, 1.0)]), Decision::NoAction);
        core.set_window(goal);
        assert_eq!(core.decide_qmdp(&[(goal, 1.0)]), Decision::Goal);
    }
}

#[test]
fn local_only_mode_keeps_the_frozen_global_policies() {
    for compact in [false, true] {
        let mut config = if compact { cfg_compact() } else { cfg() };
        config.global_sweep = false;
        let mut b = build(32);
        b.theta_cell_num = 8;
        let mut core = PlannerCore::new(b, config);
        core.prepare_goal(pose(1.2, 1.2, 0.0), &AtomicBool::new(false))
            .expect("solve");
        core.set_window(pose(0.5, 0.5, 0.0));
        let before = policies(&core);
        core.observe_scan(
            &LaserScan {
                angle_min: 0.0,
                angle_increment: 0.0,
                ranges: vec![0.2],
            },
            pose(0.5, 0.5, 0.0),
        );
        assert!(core.advance_invalidations(1));
        assert_eq!(
            policies(&core),
            before,
            "no global worker can restore discarded policies"
        );
        let inputs = core.local_inputs();
        core.refine_for(Duration::from_secs(5));
        assert_eq!(
            core.local_inputs(),
            inputs,
            "local refinement must retain the scan"
        );
    }
}

#[test]
fn compact_recenter_during_invalidation_preserves_observations_and_frozen_halos() {
    let mut core = managed_field(true);
    finish_in_background(&mut core, 100_000);
    let owner = core.claim_local();
    let mut old = core.prepare_local_job().expect("old local snapshot");
    core.cfg.scan_penalty = 20_000;
    core.observe_scan(
        &LaserScan {
            angle_min: 0.0,
            angle_increment: 0.0,
            ranges: vec![0.2],
        },
        pose(1.6, 1.6, 0.0),
    );
    let observed = core.penalty.as_ref().expect("overlay").get(18, 16);
    assert!(observed > 0);
    assert!(!core.advance_invalidations(1), "exercise a pending closure");
    core.set_window(pose(0.4, 0.4, 0.0));
    drain(&mut core);
    old.run_parallel(4).expect("old parallel pass");
    assert_eq!(core.commit_local_job(old), None);
    assert_eq!(
        core.penalty.as_ref().expect("overlay").get(18, 16),
        observed
    );
    let Field::Compact(f) = &core.cached.as_ref().expect("field").field else {
        panic!("compact fixture");
    };
    let patch = core.patch.as_ref().expect("patch");
    let (ox, oy) = patch.at.expect("hydrated");
    for s in &patch.vi.base.states {
        let (gx, gy) = (s.ix + ox, s.iy + oy);
        if gx >= 0 && gy >= 0 && gx < f.cell_num.0 && gy < f.cell_num.1 {
            assert_eq!(
                (s.total_cost, s.optimal_action.map_or(-1, |a| a as i32)),
                f.sink.read(f.orig(gx, gy, s.it)),
                "window and frozen halo must reflect invalidated sink values",
            );
        }
    }
    core.release_local(owner);
}
