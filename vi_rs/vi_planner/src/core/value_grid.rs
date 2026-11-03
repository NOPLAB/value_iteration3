//! Bounded reads for background visualization; never hold the core for a full map.
use std::ops::Range;

use super::*;

impl PlannerCore {
    /// Return complete rows, their goal epoch, and the full map height.
    pub fn value_grid_rows(
        &self,
        rows: Range<usize>,
        threshold_steps: u64,
    ) -> Option<(u64, OccupancyGrid, usize)> {
        let render = |p: &dyn PolicyView| {
            let (nx, ny, _) = p.cell_num();
            let width = nx as usize;
            let end = rows.end.min(ny as usize);
            if rows.start >= end {
                return None;
            }
            let height = end - rows.start;
            let values: Vec<_> = (rows.start..end)
                .flat_map(|y| (0..nx).map(move |x| p.value_at(x, y as i32, 0)))
                .collect();
            let (ox, oy) = p.map_origin();
            Some((
                self.global.epoch,
                OccupancyGrid {
                    width: nx,
                    height: height as i32,
                    resolution: p.xy_resolution(),
                    origin_x: ox,
                    origin_y: oy + rows.start as f64 * p.xy_resolution(),
                    origin_quat: p.map_origin_quat(),
                    data: value_slice_to_occupancy(&values, width, height, threshold_steps),
                },
                ny as usize,
            ))
        };
        match &self.cached.as_ref()?.field {
            Field::Dense(vi) => render(&vi.base),
            Field::Compact(f) => render(&f.policy()),
        }
    }
}
