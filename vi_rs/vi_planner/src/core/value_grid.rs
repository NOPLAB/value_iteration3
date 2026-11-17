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

impl PlannerCore {
    /// θ=0 全域スライスの可視化グリッド。ゴール未設定なら None。
    /// compact 経路では sink を全走査するので、広域地図では相応に重い
    /// (`publish_value_function: false` を推奨)。
    pub fn value_grid(&self, threshold_steps: u64) -> Option<OccupancyGrid> {
        match &self.cached.as_ref()?.field {
            Field::Dense(vi) => Some(value_grid_on(&vi.base, threshold_steps)),
            Field::Compact(f) => Some(value_grid_on(&f.policy(), threshold_steps)),
        }
    }

    /// ローカルウィンドウ範囲だけを現在方位の θ スライスで描画した可視化グリッド。
    /// `set_window` 後に呼ぶこと。地図端ではクランプ後の実範囲を使うので、本家
    /// `makeLocalValueFunctionMap` と違い幅とデータ長が常に一致する。
    pub fn window_value_grid(
        &self,
        pose: PoseView,
        threshold_steps: u64,
    ) -> Option<OccupancyGrid> {
        let vi = self.local()?;
        let (_, _, it) = pose_to_cell(&vi.base, pose.x, pose.y, pose.yaw_rad);
        let it = it.clamp(0, vi.base.cell_num_t - 1);
        let (x0, x1) = (vi.local_ix_min, vi.local_ix_max);
        let (y0, y1) = (vi.local_iy_min, vi.local_iy_max);
        if x1 < x0 || y1 < y0 {
            return None;
        }
        let (w, h) = ((x1 - x0 + 1) as usize, (y1 - y0 + 1) as usize);
        let mut slice = vec![0u64; w * h];
        for iy in y0..=y1 {
            for ix in x0..=x1 {
                // 形の外は MAX_COST = RViz では -1 (未知) になり、窓の形がそのまま
                // 見える。専用のセンチネルは要らない。
                slice[(iy - y0) as usize * w + (ix - x0) as usize] = if vi.in_local_area(ix, iy) {
                    vi.base.states[vi.base.to_index(ix, iy, it) as usize].total_cost
                } else {
                    MAX_COST
                };
            }
        }
        Some(OccupancyGrid {
            width: w as i32,
            height: h as i32,
            resolution: vi.base.xy_resolution,
            origin_x: vi.base.map_origin_x + x0 as f64 * vi.base.xy_resolution,
            origin_y: vi.base.map_origin_y + y0 as f64 * vi.base.xy_resolution,
            origin_quat: vi.base.map_origin_quat.clone(),
            data: value_slice_to_occupancy(&slice, w, h, threshold_steps),
        })
    }

}
