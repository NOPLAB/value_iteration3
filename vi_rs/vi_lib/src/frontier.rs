//! 2D 活性セル集合の膨張器（行スパン付き 2 パス分離膨張・世代スタンプ再利用）。
//!
//! - 各セルの入力から横方向 `±dx` を `dilate_x` へ展開しつつ行ごとの有効区間 `[x0, x1]` を記録し、
//!   縦方向 `±dy` を触った行・区間だけ `dilate_y` へ伝播させ、最後に row-major `(iy, ix)` 昇順で
//!   候補セルを回収する。
//! - 世代カウンタ (`stamp`) により毎ラウンドの配列再確保・全ゼロ初期化を避け、疎な入力でも
//!   触っていない行はスキップする。`dx >= 64` の変位にも対応する。

/// 再利用可能な 2D フロンティア膨張器。
pub struct Frontier2D {
    nx: usize,
    ny: usize,
    dilate_x: Vec<u32>,
    dilate_y: Vec<u32>,
    row_x_stamp: Vec<u32>,
    row_x0: Vec<usize>,
    row_x1: Vec<usize>,
    row_y_stamp: Vec<u32>,
    row_y0: Vec<usize>,
    row_y1: Vec<usize>,
    stamp: u32,
}

impl Frontier2D {
    /// サイズ `nx × ny` の膨張器を確保する。
    pub fn new(nx: u32, ny: u32) -> Self {
        let nx = nx as usize;
        let ny = ny as usize;
        let cells = nx * ny;
        Self {
            nx,
            ny,
            dilate_x: vec![0; cells],
            dilate_y: vec![0; cells],
            row_x_stamp: vec![0; ny],
            row_x0: vec![0; ny],
            row_x1: vec![0; ny],
            row_y_stamp: vec![0; ny],
            row_y0: vec![0; ny],
            row_y1: vec![0; ny],
            stamp: 0,
        }
    }

    /// `cells` を L∞ 箱半径 `(dx, dy)` で膨張し、row-major `(iy, ix)` 昇順・重複なしで返す。
    pub fn dilate(&mut self, cells: &[(u32, u32)], dx: u32, dy: u32) -> Vec<(u32, u32)> {
        let mut out = Vec::with_capacity(cells.len());
        self.dilate_into(cells, dx, dy, &mut out);
        out
    }

    /// `cells` を L∞ 箱半径 `(dx, dy)` で膨張し、`out` をクリアしてから row-major `(iy, ix)` 昇順で書き込む。
    pub fn dilate_into(
        &mut self,
        cells: &[(u32, u32)],
        dx: u32,
        dy: u32,
        out: &mut Vec<(u32, u32)>,
    ) {
        out.clear();
        if cells.is_empty() || self.nx == 0 || self.ny == 0 {
            return;
        }
        self.stamp = self.stamp.wrapping_add(1);
        if self.stamp == 0 {
            self.dilate_x.fill(0);
            self.dilate_y.fill(0);
            self.row_x_stamp.fill(0);
            self.row_y_stamp.fill(0);
            self.stamp = 1;
        }
        let stamp = self.stamp;
        let nx = self.nx;
        let ny = self.ny;
        let dx = dx as usize;
        let dy = dy as usize;

        let mut min_y = ny - 1;
        let mut max_y = 0usize;

        // Pass 1: 横方向 (±dx) を dilate_x へ展開し、行ごとの有効区間 [x0, x1] を記録する。
        for &(ixu, iyu) in cells {
            let ix = ixu as usize;
            let iy = iyu as usize;
            debug_assert!(ix < nx && iy < ny);
            let x0 = ix.saturating_sub(dx);
            let x1 = (ix + dx).min(nx - 1);
            if iy < min_y {
                min_y = iy;
            }
            if iy > max_y {
                max_y = iy;
            }
            if self.row_x_stamp[iy] != stamp {
                self.row_x_stamp[iy] = stamp;
                self.row_x0[iy] = x0;
                self.row_x1[iy] = x1;
            } else {
                if x0 < self.row_x0[iy] {
                    self.row_x0[iy] = x0;
                }
                if x1 > self.row_x1[iy] {
                    self.row_x1[iy] = x1;
                }
            }
            let row = &mut self.dilate_x[iy * nx..(iy + 1) * nx];
            row[x0..=x1].fill(stamp);
        }

        // Pass 2: 触った行・区間だけ縦方向 (±dy) を dilate_y へ伝播する。
        let out_y0 = min_y.saturating_sub(dy);
        let out_y1 = (max_y + dy).min(ny - 1);
        for iy in min_y..=max_y {
            if self.row_x_stamp[iy] != stamp {
                continue;
            }
            let x0 = self.row_x0[iy];
            let x1 = self.row_x1[iy];
            let src_start = iy * nx;
            let y0 = iy.saturating_sub(dy);
            let y1 = (iy + dy).min(ny - 1);
            for oy in y0..=y1 {
                if self.row_y_stamp[oy] != stamp {
                    self.row_y_stamp[oy] = stamp;
                    self.row_y0[oy] = x0;
                    self.row_y1[oy] = x1;
                } else {
                    if x0 < self.row_y0[oy] {
                        self.row_y0[oy] = x0;
                    }
                    if x1 > self.row_y1[oy] {
                        self.row_y1[oy] = x1;
                    }
                }
                let dst_start = oy * nx;
                for ix in x0..=x1 {
                    if self.dilate_x[src_start + ix] == stamp {
                        self.dilate_y[dst_start + ix] = stamp;
                    }
                }
            }
        }

        // Pass 3: 触った出力行・区間を row-major (iy, ix) 昇順で回収する。
        for oy in out_y0..=out_y1 {
            if self.row_y_stamp[oy] != stamp {
                continue;
            }
            let x0 = self.row_y0[oy];
            let x1 = self.row_y1[oy];
            let row = &self.dilate_y[oy * nx..(oy + 1) * nx];
            let oyu = oy as u32;
            for (ix, &v) in row.iter().enumerate().take(x1 + 1).skip(x0) {
                if v == stamp {
                    out.push((ix as u32, oyu));
                }
            }
        }
    }
}

/// 1 回限りの 2D 膨張ヘルパ（初期フロンティア種付け等）。
pub fn dilate_2d_once(nx: u32, ny: u32, cells: &[(u32, u32)], dx: u32, dy: u32) -> Vec<(u32, u32)> {
    Frontier2D::new(nx, ny).dilate(cells, dx, dy)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_input_returns_empty() {
        let mut f = Frontier2D::new(10, 10);
        assert!(f.dilate(&[], 2, 2).is_empty());
    }

    #[test]
    fn identity_at_zero_deduplicates_and_sorts_row_major() {
        let mut f = Frontier2D::new(10, 10);
        let out = f.dilate(&[(7, 5), (2, 2), (7, 5), (0, 9), (1, 2)], 0, 0);
        assert_eq!(out, vec![(1, 2), (2, 2), (7, 5), (0, 9)]);
    }

    #[test]
    fn single_cell_expands_to_box_in_row_major_order() {
        let mut f = Frontier2D::new(5, 5);
        let out = f.dilate(&[(2, 2)], 1, 1);
        let mut expected = Vec::new();
        for y in 1..=3 {
            for x in 1..=3 {
                expected.push((x, y));
            }
        }
        assert_eq!(out, expected);
    }

    #[test]
    fn clamps_to_grid_bounds_and_supports_large_dx() {
        let mut f = Frontier2D::new(80, 4);
        let out = f.dilate(&[(3, 0)], 70, 1);
        assert_eq!(out.len(), 74 * 2);
        assert_eq!(out.first(), Some(&(0, 0)));
        assert_eq!(out.last(), Some(&(73, 1)));
    }

    #[test]
    fn consecutive_calls_and_stamp_wraparound_are_clean() {
        let mut f = Frontier2D::new(8, 8);
        f.stamp = u32::MAX;
        let a = f.dilate(&[(0, 0)], 1, 0);
        assert_eq!(a, vec![(0, 0), (1, 0)]);
        assert_eq!(f.stamp, 1);
        let b = f.dilate(&[(7, 7)], 0, 1);
        assert_eq!(b, vec![(7, 6), (7, 7)]);
    }
}
