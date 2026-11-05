//! Remove obsolete policy dependencies before repairing an increased cost.
use std::collections::VecDeque;

use super::*;

#[derive(Clone, Copy)]
struct Predecessor {
    dx: i32,
    dy: i32,
    theta: i32,
    action: usize,
}

#[derive(Default)]
pub(super) struct Invalidation {
    reverse: Vec<Vec<Predecessor>>,
    pub(super) pending: VecDeque<(i32, i32, i32)>,
}

/// No action on a free nonterminal state means its old value is unavailable.
/// Keep that committed value readable until repair, but never calculate with it.
pub(super) fn repair_values(vi: &mut ValueIterator) {
    for s in &mut vi.states {
        if s.policy_invalidated() {
            s.total_cost = MAX_COST;
        }
    }
}

impl PlannerCore {
    pub(super) fn queue_cost_increases(&mut self, cells: Vec<(i32, i32)>) {
        // Local-only mode deliberately keeps the static global field frozen;
        // there is no worker that could restore discarded far-field policies.
        if cells.is_empty() || !self.cfg.global_sweep {
            return;
        }
        if self.global.invalidation.reverse.is_empty() {
            let Some(local) = self.local() else { return };
            let nt = local.base.cell_num_t;
            let mut reverse = vec![Vec::new(); nt as usize];
            for (action, a) in local.base.actions.iter().enumerate() {
                for (theta, transitions) in a.state_transitions.iter().enumerate() {
                    for t in transitions {
                        if t.prob > 0 {
                            reverse[t.dit.rem_euclid(nt) as usize].push(Predecessor {
                                dx: t.dix,
                                dy: t.diy,
                                theta: theta as i32,
                                action,
                            });
                        }
                    }
                }
            }
            self.global.invalidation.reverse = reverse;
        }
        let nt = self.global.invalidation.reverse.len() as i32;
        for (x, y) in cells {
            for theta in 0..nt {
                self.global.invalidation.pending.push_back((x, y, theta));
            }
        }
        // Keep outstanding IDs until their commits return them. Their old
        // epoch rejects stale halos; global commits requeue the checked-out work.
        self.global.epoch += 1;
        self.global.boundary_revision += 1;
        self.global.certified = false;
    }

    /// No Bellman work may run until the old-policy closure has been removed.
    /// A batch bounds mutex hold time without allocating a full-map reverse graph.
    pub(in crate::core) fn advance_invalidations(&mut self, budget: usize) -> bool {
        let (nx, ny) = (self.build.grid.width, self.build.grid.height);
        for _ in 0..budget {
            let Some((x, y, theta)) = self.global.invalidation.pending.pop_front() else {
                return true;
            };
            let len = self.global.invalidation.reverse[theta as usize].len();
            for i in 0..len {
                let p = self.global.invalidation.reverse[theta as usize][i];
                let (px, py) = (x - p.dx, y - p.dy);
                if px < 0 || py < 0 || px >= nx || py >= ny {
                    continue;
                }
                if self.invalidate_policy_state((px, py, p.theta), p.action) {
                    self.global
                        .invalidation
                        .pending
                        .push_back((px, py, p.theta));
                    self.enqueue_global((px, px, py, py));
                }
            }
        }
        self.global.invalidation.pending.is_empty()
    }

    /// Test the stored action before clearing it. Cleared actions also serve as
    /// the visited set: a state is propagated at most once per invalidation phase.
    fn invalidate_policy_state(&mut self, (x, y, theta): (i32, i32, i32), action: usize) -> bool {
        let Some(cached) = self.cached.as_mut() else {
            return false;
        };
        match &mut cached.field {
            Field::Dense(vi) => {
                let idx = vi.base.to_index(x, y, theta) as usize;
                let s = &mut vi.base.states[idx];
                if !s.free || s.final_state || s.optimal_action != Some(action) {
                    return false;
                }
                s.optimal_action = None;
            }
            Field::Compact(f) => {
                let idx = f.orig(x, y, theta);
                let (value, a) = f.sink.read(idx);
                // Compact goals are exactly the zero-valued absorbing states.
                if value == 0 || a != action as i32 {
                    return false;
                }
                f.sink.write_column(idx, &[value], &[-1]);
                // Invalidate the entire hydrated footprint, including the frozen
                // halo; a later local commit must not restore an old boundary.
                if let Some(p) = self.patch.as_mut() {
                    if let Some((ox, oy)) = p.at {
                        if p.vi.base.in_map_area(x - ox, y - oy) {
                            let idx = p.vi.base.to_index(x - ox, y - oy, theta) as usize;
                            let s = &mut p.vi.base.states[idx];
                            s.optimal_action = None;
                        }
                    }
                }
            }
        }
        true
    }
}
