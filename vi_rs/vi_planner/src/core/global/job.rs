//! Worker-owned state; Bellman updates never borrow the manager.
use vi_lib::state::State;

use super::super::ValueIterator;

pub struct GlobalJob {
    pub(super) epoch: u64,
    pub(super) id: u64,
    pub(super) vi: ValueIterator,
    pub(super) before: Vec<State>,
    pub(super) origin: (i32, i32),
    pub(super) cells: Vec<(i32, i32)>,
    pub(super) compact: bool,
    pub(super) reverse: (bool, bool),
    pub(super) delta: u64,
    pub(super) ran: bool,
}

impl GlobalJob {
    /// Update only the assigned cells; the transition halo remains frozen.
    pub fn run(&mut self) {
        if self.ran {
            return;
        }
        let pass = |vi: &mut ValueIterator, cells: &[(i32, i32)]| {
            let mut delta = 0u64;
            for &(x, y) in cells {
                for t in 0..vi.cell_num_t {
                    delta =
                        delta.saturating_add(vi.value_iteration_at(vi.to_index(x, y, t) as usize));
                }
            }
            delta
        };
        if self.compact {
            self.cells.sort_by_key(|&(x, y)| {
                (
                    if self.reverse.1 { -y } else { y },
                    if self.reverse.0 { -x } else { x },
                )
            });
        }
        self.delta = pass(&mut self.vi, &self.cells);
        if self.compact && self.delta > 0 {
            self.cells.reverse();
            self.delta = self.delta.saturating_add(pass(&mut self.vi, &self.cells));
        }
        self.ran = true;
    }
}

/// Copy geometry and transitions, never full-map states or sweep orders.
pub(super) fn geometry(base: &ValueIterator, nx: i32, ny: i32) -> ValueIterator {
    let mut vi = ValueIterator::new(base.actions.clone(), 1);
    vi.cell_num_x = nx;
    vi.cell_num_y = ny;
    vi.cell_num_t = base.cell_num_t;
    vi.xy_resolution = base.xy_resolution;
    vi.t_resolution = base.t_resolution;
    vi
}
