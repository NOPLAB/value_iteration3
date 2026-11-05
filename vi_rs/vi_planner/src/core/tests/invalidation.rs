//! Independent forward-policy oracle for the reverse invalidation traversal.
use super::*;
use std::collections::BTreeSet;

pub(super) fn policies(core: &PlannerCore) -> Vec<(u64, i32)> {
    match &core.cached.as_ref().expect("field").field {
        Field::Dense(vi) => vi
            .base
            .states
            .iter()
            .map(|s| (s.total_cost, s.optimal_action.map_or(-1, |a| a as i32)))
            .collect(),
        Field::Compact(f) => {
            let n = f.cell_num.0 as usize * f.cell_num.1 as usize * f.cell_num.2 as usize;
            (0..n).map(|i| f.sink.read(i)).collect()
        }
    }
}

pub(super) fn drain(core: &mut PlannerCore) -> Duration {
    let mut longest = Duration::ZERO;
    loop {
        let start = Instant::now();
        let done = core.advance_invalidations(512);
        longest = longest.max(start.elapsed());
        if done {
            return longest;
        }
    }
}

#[test]
fn increased_cost_invalidates_exact_stochastic_policy_closure_on_scaled_grids() {
    for compact in [false, true] {
        for scale in [1, 2] {
            let mut core = managed_field_scaled(compact, scale);
            let scan = LaserScan {
                angle_min: 0.0,
                angle_increment: 0.0,
                ranges: vec![0.2],
            };
            core.cfg.scan_penalty = 0;
            core.observe_scan(&scan, pose(1.6, 1.6, 0.0));
            finish_in_background(&mut core, 100_000);
            let before = policies(&core);
            let old_inputs = core.local_inputs();
            let local = core.local().expect("window");
            let nt = local.base.cell_num_t;
            let actions = local.base.actions.clone();
            let p0 = core.patch.as_ref().and_then(|p| p.at).unwrap_or((0, 0));
            let coordinates: Vec<_> = (local.local_iy_min..=local.local_iy_max)
                .flat_map(|y| {
                    (local.local_ix_min..=local.local_ix_max).filter_map(move |x| {
                        local.in_local_area(x, y).then_some((x + p0.0, y + p0.1))
                    })
                })
                .collect();

            core.cfg.scan_penalty = 20_000;
            core.observe_scan(&scan, pose(1.6, 1.6, 0.0));
            let inputs = core.local_inputs();
            let increased: BTreeSet<_> = coordinates
                .into_iter()
                .zip(old_inputs.iter().zip(&inputs))
                .filter_map(|(xy, (old, new))| (new.1 + new.2 > old.1 + old.2).then_some(xy))
                .collect();
            assert!(
                !increased.is_empty(),
                "fixture must increase an observed input"
            );
            let (nx, ny) = (core.build.grid.width, core.build.grid.height);
            assert_eq!((nx, ny, nt), (32, 32, 8));
            let index = |x: i32, y: i32, t: i32| (t + x * nt + y * nx * nt) as usize;
            let mut affected = BTreeSet::new();
            let mut depth = 0;
            // Forward-read every old chosen action. This oracle deliberately
            // does not use the production reverse index or queue.
            loop {
                let mut next = affected.clone();
                for y in 0..ny {
                    for x in 0..nx {
                        for theta in 0..nt {
                            let (value, a) = before[index(x, y, theta)];
                            if a < 0 || value == 0 {
                                continue;
                            }
                            if actions[a as usize].state_transitions[theta as usize]
                                .iter()
                                .any(|t| {
                                    let target = (x + t.dix, y + t.diy, t.dit.rem_euclid(nt));
                                    t.prob > 0
                                        && (increased.contains(&(target.0, target.1))
                                            || affected.contains(&target))
                                })
                            {
                                next.insert((x, y, theta));
                            }
                        }
                    }
                }
                if next == affected {
                    break;
                }
                affected = next;
                depth += 1;
            }
            assert!(depth > 1, "fixture must exercise transitive predecessors");
            assert!(affected.iter().any(|&(_, _, t)| t != 0));
            assert!(
                affected.iter().any(|&(x, y, t)| {
                    let a = before[index(x, y, t)].1 as usize;
                    actions[a].state_transitions[t as usize]
                        .iter()
                        .filter(|tr| tr.prob > 0)
                        .count()
                        > 1
                }),
                "fixture must exercise probabilistic successors"
            );
            let longest = drain(&mut core);
            let after = policies(&core);
            let mut unchanged = 0;
            for y in 0..ny {
                for x in 0..nx {
                    for theta in 0..nt {
                        let idx = index(x, y, theta);
                        if affected.contains(&(x, y, theta)) {
                            assert_eq!(
                                after[idx],
                                (before[idx].0, -1),
                                "{compact} {scale} ({x},{y},{theta})"
                            );
                        } else {
                            unchanged += 1;
                            assert_eq!(after[idx], before[idx], "unaffected policy was discarded");
                        }
                    }
                }
            }
            assert!(unchanged > 0);
            assert_eq!(
                core.local_inputs(),
                inputs,
                "invalidation must preserve observations"
            );
            eprintln!("invalidation compact={compact} scale={scale} affected={} max_512_target_batch={longest:?}", affected.len());
        }
    }
}

#[test]
fn decreased_scan_cost_repairs_without_discarding_policies() {
    for compact in [false, true] {
        let mut core = managed_field(compact);
        finish_in_background(&mut core, 100_000);
        let before = policies(&core);
        core.cfg.scan_penalty = 1;
        core.observe_scan(
            &LaserScan {
                angle_min: 0.0,
                angle_increment: 0.0,
                ranges: vec![0.2],
            },
            pose(1.6, 1.6, 0.0),
        );
        assert!(
            core.advance_invalidations(1),
            "a decrease must not start invalidation"
        );
        assert_eq!(policies(&core), before);
        finish_in_background(&mut core, 100_000);
        let after = policies(&core);
        assert!(after.iter().zip(&before).all(|(a, b)| a.0 <= b.0));
        assert!(after.iter().zip(&before).any(|(a, b)| a.0 < b.0));
    }
}

#[test]
fn increased_scan_cost_rejects_both_outstanding_workers_without_republishing_halos() {
    for compact in [false, true] {
        let mut core = managed_field(compact);
        finish_in_background(&mut core, 100_000);
        let owner = core.claim_local();
        core.restart_global_check();
        let mut global = core.prepare_global_job(128).expect("global snapshot");
        let mut local = core.prepare_local_job().expect("local snapshot");
        core.cfg.scan_penalty = 20_000;
        core.observe_scan(
            &LaserScan {
                angle_min: 0.0,
                angle_increment: 0.0,
                ranges: vec![0.2],
            },
            pose(1.6, 1.6, 0.0),
        );
        drain(&mut core);
        let invalidated = policies(&core);
        local.run_parallel(4).expect("parallel stale pass");
        assert_eq!(core.commit_local_job(local), None);
        global.run();
        assert_eq!(core.commit_global_job(global).0, 0);
        assert_eq!(
            policies(&core),
            invalidated,
            "stale snapshot restored invalidated values"
        );
        assert!(
            core.prepare_local_job().is_some(),
            "old checkout must release its slot"
        );
        core.release_local(owner);
        finish_in_background(&mut core, 100_000);
        assert!(
            !core.is_dirty(),
            "discarded global work must be rescheduled"
        );
    }
}
