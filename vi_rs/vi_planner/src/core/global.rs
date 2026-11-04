//! Manager-owned scheduling and bounded worker-owned Bellman snapshots.
use std::collections::BTreeSet;

use vi_lib::state::State;

use super::*;

mod job;
mod local;
mod ownership;
use job::geometry;
pub use job::GlobalJob;
pub use local::LocalJob;

#[derive(Default)]
pub(super) struct Manager {
    pub(super) epoch: u64,
    serial: u64,
    owner: Option<u64>,
    owned: BTreeSet<(i32, i32)>,
    pub(super) deferred: BTreeSet<(i32, i32)>,
    pub(super) certified: bool,
    pub(super) in_flight: Option<u64>,
    pub(super) local_in_flight: Option<u64>,
    local_revision: u64,
    boundary_revision: u64,
}

impl Manager {
    pub(super) fn local_pending(&self) -> bool {
        !self.owned.is_empty() && !self.certified
    }
}

fn unchanged(a: &State, b: &State) -> bool {
    a.total_cost == b.total_cost
        && a.optimal_action == b.optimal_action
        && a.free == b.free
        && a.final_state == b.final_state
        && a.penalty == b.penalty
        && a.local_penalty == b.local_penalty
}

impl PlannerCore {
    /// Prepare one bounded batch. None may mean only locally deferred work remains.
    /// Exactly one job may be outstanding; it must be returned to commit.
    pub fn prepare_global_job(&mut self, max_states: usize) -> Option<GlobalJob> {
        if self.global.in_flight.is_some() || self.cached.is_none() {
            return None;
        }
        let (mut vi, origin, candidates, compact, reverse) = if self.cfg.use_compact() {
            let Some(r) = self.repair.as_mut() else {
                self.dirty = false;
                return None;
            };
            let Some(t) = r.queue.pop_front() else {
                self.settle_global();
                return None;
            };
            r.queued[t as usize] = false;
            r.visits += 1;
            let Field::Compact(f) = &self.cached.as_ref()?.field else {
                return None;
            };
            let (x0, x1, y0, y1) = r.interior_of(t, f.cell_num.0, f.cell_num.1);
            let p0 = (x0 - r.halo, y0 - r.halo);
            compact::hydrate_states(&mut r.vi, f, &self.build, self.penalty.as_ref(), p0);
            let mut vi = geometry(&r.vi, r.vi.cell_num_x, r.vi.cell_num_y);
            vi.states = r.vi.states.clone();
            let cells = (y0..=y1)
                .flat_map(|y| (x0..=x1).map(move |x| (x, y)))
                .collect();
            (vi, p0, cells, true, (r.visits & 1 != 0, r.visits & 2 != 0))
        } else {
            let Field::Dense(base) = &self.cached.as_ref()?.field else {
                return None;
            };
            let base = &base.base;
            let sw = &mut self.sweep;
            let reach = *sw.reach.get_or_insert_with(|| {
                let r = transition_reach(base);
                assert!((1..64).contains(&r));
                r as u32
            });
            if sw.pos >= sw.cands.len() {
                sw.cands.clear();
                sw.pos = 0;
                if let Some(seed) = sw.pending.take() {
                    sw.cands.extend(seed.dilate(reach, reach).enumerate());
                }
            }
            if sw.cands.is_empty() {
                self.settle_global();
                return None;
            }
            let budget = (max_states / base.cell_num_t.max(1) as usize).max(1);
            let (x, y) = sw.cands[sw.pos];
            let (mut x0, mut x1, mut y0, mut y1) = (x as i32, x as i32, y as i32, y as i32);
            let mut cells = Vec::new();
            while sw.pos < sw.cands.len() && cells.len() < budget {
                let (x, y) = sw.cands[sw.pos];
                let (x, y) = (x as i32, y as i32);
                let bbox = (x0.min(x), x1.max(x), y0.min(y), y1.max(y));
                // A sparse active set must never turn into a full-map snapshot.
                if bbox.1 - bbox.0 >= 32 || bbox.3 - bbox.2 >= 32 {
                    break;
                }
                (x0, x1, y0, y1) = bbox;
                cells.push((x, y));
                sw.pos += 1;
            }
            x0 = (x0 - reach as i32).max(0);
            y0 = (y0 - reach as i32).max(0);
            x1 = (x1 + reach as i32).min(base.cell_num_x - 1);
            y1 = (y1 + reach as i32).min(base.cell_num_y - 1);
            let mut vi = geometry(base, x1 - x0 + 1, y1 - y0 + 1);
            for y in y0..=y1 {
                for x in x0..=x1 {
                    for t in 0..base.cell_num_t {
                        let mut s = base.states[base.to_index(x, y, t) as usize].clone();
                        s.ix -= x0;
                        s.iy -= y0;
                        vi.states.push(s);
                    }
                }
            }
            (vi, (x0, y0), cells, false, (false, false))
        };
        // The live patch also carries free/static-penalty edits not represented
        // by the compact sink. Keep them in both the interior and full halo.
        if compact {
            if let Some(p) = self.patch.as_ref() {
                if let Some(p0) = p.at {
                    for s in &mut vi.states {
                        let (x, y) = (s.ix + origin.0 - p0.0, s.iy + origin.1 - p0.1);
                        if p.vi.base.in_map_area(x, y) {
                            let live = &p.vi.base.states[p.vi.base.to_index(x, y, s.it) as usize];
                            s.free = live.free;
                            s.penalty = live.penalty;
                            s.local_penalty = live.local_penalty;
                        }
                    }
                }
            }
        }
        let mut cells = Vec::new();
        for (x, y) in candidates {
            if self.global.owned.contains(&(x, y)) {
                if !self.global.certified {
                    self.global.deferred.insert((x, y));
                }
            } else {
                cells.push((x - origin.0, y - origin.1));
            }
        }
        if !compact {
            self.sweep.visits += cells.len();
        }
        self.global.serial += 1;
        let id = self.global.serial;
        self.global.in_flight = Some(id);
        self.dirty = true;
        Some(GlobalJob {
            epoch: self.global.epoch,
            id,
            before: vi.states.clone(),
            vi,
            origin,
            cells,
            compact,
            reverse,
            delta: 0,
            ran: false,
        })
    }

    pub fn commit_global_job(&mut self, job: GlobalJob) -> (u64, bool) {
        if job.epoch != self.global.epoch || self.global.in_flight != Some(job.id) {
            return (0, !self.is_dirty());
        }
        self.global.in_flight = None;
        let nt = job.vi.cell_num_t;
        let mut changed = Vec::new();
        let mut retry = Vec::new();
        for &(x, y) in &job.cells {
            let (gx, gy) = (x + job.origin.0, y + job.origin.1);
            if self.global.owned.contains(&(gx, gy)) {
                if !self.global.certified {
                    self.global.deferred.insert((gx, gy));
                }
                continue;
            }
            if !job.ran {
                retry.push((gx, gy));
                continue;
            }
            let i = job.vi.to_index(x, y, 0) as usize;
            match &mut self.cached.as_mut().unwrap().field {
                Field::Dense(vi) => {
                    let dst = vi.base.to_index(gx, gy, 0) as usize;
                    if !(0..nt as usize)
                        .all(|t| unchanged(&vi.base.states[dst + t], &job.before[i + t]))
                    {
                        retry.push((gx, gy));
                        continue;
                    }
                    let mut moved = false;
                    for t in 0..nt as usize {
                        let s = &job.vi.states[i + t];
                        let live = &mut vi.base.states[dst + t];
                        moved |= live.total_cost != s.total_cost;
                        live.total_cost = s.total_cost;
                        live.optimal_action = s.optimal_action;
                    }
                    if moved {
                        changed.push((gx, gy));
                    }
                }
                Field::Compact(f) => {
                    let dst = f.orig(gx, gy, 0);
                    if !(0..nt as usize).all(|t| {
                        f.sink.read(dst + t)
                            == (
                                job.before[i + t].total_cost,
                                job.before[i + t].optimal_action.map_or(-1, |a| a as i32),
                            )
                    }) {
                        retry.push((gx, gy));
                        continue;
                    }
                    let values: Vec<_> = job.vi.states[i..i + nt as usize]
                        .iter()
                        .map(|s| s.total_cost)
                        .collect();
                    let actions: Vec<_> = job.vi.states[i..i + nt as usize]
                        .iter()
                        .map(|s| s.optimal_action.map_or(-1, |a| a as i32))
                        .collect();
                    if (0..nt as usize).any(|t| f.sink.read(dst + t) != (values[t], actions[t])) {
                        f.sink.write_column(dst, &values, &actions);
                        changed.push((gx, gy));
                    }
                }
            }
        }
        for &(x, y) in changed.iter().chain(&retry) {
            self.enqueue_global((x, x, y, y));
        }
        if !changed.is_empty() {
            // A new boundary value invalidates the local fixed-point certificate.
            self.global.boundary_revision += 1;
            self.global.certified = false;
            self.refresh_local_boundary();
        }
        (job.delta, self.settle_global())
    }
}
