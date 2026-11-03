//! Local ownership, deferred invalidations, and whole-field completion.
use super::*;

impl PlannerCore {
    /// Latest follow claim wins. A superseded thread cannot release its successor.
    pub fn claim_local(&mut self) -> u64 {
        self.global.serial += 1;
        let owner = self.global.serial;
        self.global.owner = Some(owner);
        self.global.certified = false;
        self.update_local_ownership();
        owner
    }

    pub fn release_local(&mut self, owner: u64) {
        if self.global.owner == Some(owner) {
            self.global.owner = None;
            self.update_local_ownership();
        }
    }

    pub(in crate::core) fn invalidate_global_jobs(&mut self) {
        self.global.epoch += 1;
        self.global.in_flight = None;
        self.global.owned.clear();
        self.global.deferred.clear();
        self.global.certified = false;
        self.dirty = false;
        self.sweep.clear();
        if let Some(r) = self.repair.as_mut() {
            r.clear();
        }
    }

    pub(in crate::core) fn update_local_ownership(&mut self) {
        let mut owned = BTreeSet::new();
        if self.global.owner.is_some() {
            if let Some(vi) = self.local() {
                let p0 = if self.cfg.use_compact() {
                    self.patch.as_ref().and_then(|p| p.at).unwrap_or((0, 0))
                } else {
                    (0, 0)
                };
                for y in vi.local_iy_min..=vi.local_iy_max {
                    for x in vi.local_ix_min..=vi.local_ix_max {
                        if vi.in_local_area(x, y) {
                            let g = (x + p0.0, y + p0.1);
                            if g.0 >= 0
                                && g.1 >= 0
                                && g.0 < self.build.grid.width
                                && g.1 < self.build.grid.height
                            {
                                owned.insert(g);
                            }
                        }
                    }
                }
            }
        }
        if owned != self.global.owned {
            self.global.certified = false;
        }
        self.global.owned = owned;
        let released: Vec<_> = self
            .global
            .deferred
            .iter()
            .copied()
            .filter(|p| !self.global.owned.contains(p))
            .collect();
        for (x, y) in released {
            self.global.deferred.remove(&(x, y));
            self.enqueue_global((x, x, y, y));
        }
    }

    pub(in crate::core) fn certify_local(&mut self) {
        self.global.certified = true;
        self.global.deferred.clear();
    }

    pub(super) fn refresh_local_boundary(&mut self) {
        let Some(p) = self.patch.as_mut() else { return };
        let Some(p0) = p.at else { return };
        let Some(c) = self.cached.as_ref() else {
            return;
        };
        let Field::Compact(f) = &c.field else { return };
        for s in &mut p.vi.base.states {
            let g = (s.ix + p0.0, s.iy + p0.1);
            if g.0 < 0
                || g.1 < 0
                || g.0 >= f.cell_num.0
                || g.1 >= f.cell_num.1
                || self.global.owned.contains(&g)
            {
                continue;
            }
            let (value, action) = f.sink.read(f.orig(g.0, g.1, s.it));
            s.total_cost = value;
            s.optimal_action = (action >= 0).then_some(action as usize);
        }
    }

    pub(super) fn enqueue_global(&mut self, rect: (i32, i32, i32, i32)) {
        self.dirty = true;
        if let Some(r) = self.repair.as_mut() {
            r.enqueue_around(rect);
        } else {
            self.sweep
                .seed_rect(self.build.grid.width, self.build.grid.height, rect);
        }
    }

    /// Scan and margin updates are theta-uniform: read only theta=0 in the window.
    pub(in crate::core) fn local_inputs(&self) -> Vec<(bool, u64, u64)> {
        let Some(vi) = self.local() else {
            return Vec::new();
        };
        (vi.local_iy_min..=vi.local_iy_max)
            .flat_map(|y| {
                (vi.local_ix_min..=vi.local_ix_max).filter_map(move |x| {
                    if !vi.in_local_area(x, y) {
                        return None;
                    }
                    let s = &vi.base.states[vi.base.to_index(x, y, 0) as usize];
                    Some((s.free, s.penalty, s.local_penalty))
                })
            })
            .collect()
    }

    /// Changed inputs activate predecessors even when the local value did not move.
    pub(in crate::core) fn invalidate_local_inputs(&mut self, before: Vec<(bool, u64, u64)>) {
        if before == self.local_inputs() {
            return;
        }
        self.global.certified = false;
        let Some(vi) = self.local() else { return };
        let p0 = if self.cfg.use_compact() {
            self.patch.as_ref().and_then(|p| p.at).unwrap_or((0, 0))
        } else {
            (0, 0)
        };
        let rect = (
            vi.local_ix_min + p0.0,
            vi.local_ix_max + p0.0,
            vi.local_iy_min + p0.1,
            vi.local_iy_max + p0.1,
        );
        self.enqueue_global(rect);
    }

    pub(super) fn settle_global(&mut self) -> bool {
        let queued = if let Some(r) = self.repair.as_ref() {
            !r.queue.is_empty()
        } else {
            self.sweep.pos < self.sweep.cands.len()
                || self
                    .sweep
                    .pending
                    .as_ref()
                    .is_some_and(|p| p.popcount() > 0)
        };
        let done = !queued && self.global.in_flight.is_none() && self.global.deferred.is_empty();
        self.dirty = !done;
        if done {
            if let Some(r) = self.repair.as_mut() {
                if r.visits > 0 {
                    r.settle();
                }
            } else if self.sweep.visits > 0 {
                self.sweep.settle();
            }
            if let Some(c) = self.cached.as_mut() {
                c.mark_complete();
            }
        }
        done
    }
}
