//! One local pass over a bounded snapshot, computed without the core lock.
use super::*;

pub struct LocalJob {
    epoch: u64,
    owner: u64,
    id: u64,
    revision: u64,
    boundary_revision: u64,
    vi: ValueIterator,
    origin: (i32, i32),
    cells: Vec<(i32, i32)>,
    delta: u64,
    ran: bool,
}

impl LocalJob {
    /// Complete the captured local mask once. The transition halo stays frozen.
    /// Pacing belongs to the worker; never restart a partial pass at column zero.
    pub fn run(&mut self) {
        if self.ran {
            return;
        }
        for &(x, y) in &self.cells {
            for t in 0..self.vi.cell_num_t {
                let i = self.vi.to_index(x, y, t) as usize;
                self.delta = self.delta.saturating_add(self.vi.value_iteration_at(i));
            }
        }
        self.ran = true;
    }
}

impl PlannerCore {
    /// Checkout one local pass while keeping live scan inputs and policy readable.
    /// Exactly one local job may be outstanding; return it even if it was not run.
    pub fn prepare_local_job(&mut self) -> Option<LocalJob> {
        let owner = self.global.owner?;
        if self.global.local_in_flight.is_some() || self.global.owned.is_empty() {
            return None;
        }
        let local = self.local()?;
        let base = &local.base;
        let reach = transition_reach(base);
        let x0 = (local.local_ix_min - reach).max(0);
        let y0 = (local.local_iy_min - reach).max(0);
        let x1 = (local.local_ix_max + reach).min(base.cell_num_x - 1);
        let y1 = (local.local_iy_max + reach).min(base.cell_num_y - 1);
        let p0 = self.patch.as_ref().and_then(|p| p.at).unwrap_or((0, 0));
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
        let mut cells = Vec::new();
        for x in local.local_ix_min..=local.local_ix_max {
            for y in local.local_iy_min..=local.local_iy_max {
                if local.in_local_area(x, y) && self.global.owned.contains(&(x + p0.0, y + p0.1)) {
                    cells.push((x - x0, y - y0));
                }
            }
        }
        self.global.serial += 1;
        let id = self.global.serial;
        self.global.local_in_flight = Some(id);
        Some(LocalJob {
            epoch: self.global.epoch,
            owner,
            id,
            revision: self.global.local_revision,
            boundary_revision: self.global.boundary_revision,
            vi,
            origin: (p0.0 + x0, p0.1 + y0),
            cells,
            delta: 0,
            ran: false,
        })
    }

    /// Publish only values and actions. None means stale or unexecuted work.
    /// A changed global boundary permits progress, but not a local certificate.
    pub fn commit_local_job(&mut self, job: LocalJob) -> Option<u64> {
        if job.epoch != self.global.epoch
            || self.global.owner != Some(job.owner)
            || self.global.local_in_flight != Some(job.id)
        {
            return None;
        }
        self.global.local_in_flight = None;
        if !job.ran || job.revision != self.global.local_revision {
            return None;
        }
        let p0 = self.patch.as_ref().and_then(|p| p.at).unwrap_or((0, 0));
        let local = self.local_mut()?;
        let mut changed = Vec::new();
        for &(x, y) in &job.cells {
            let (gx, gy) = (x + job.origin.0, y + job.origin.1);
            let dst = local.base.to_index(gx - p0.0, gy - p0.1, 0) as usize;
            let src = job.vi.to_index(x, y, 0) as usize;
            let mut moved = false;
            for t in 0..job.vi.cell_num_t as usize {
                let s = &job.vi.states[src + t];
                let live = &mut local.base.states[dst + t];
                moved |= live.total_cost != s.total_cost || live.optimal_action != s.optimal_action;
                live.total_cost = s.total_cost;
                live.optimal_action = s.optimal_action;
            }
            if moved {
                changed.push((gx, gy));
            }
        }
        if self.cfg.use_compact() {
            self.commit_window();
        } else {
            for (x, y) in changed {
                self.enqueue_global((x, x, y, y));
            }
        }
        self.global.certified = false;
        if job.delta == 0 && job.boundary_revision == self.global.boundary_revision {
            self.certify_local();
        }
        self.settle_global();
        Some(job.delta)
    }
}
