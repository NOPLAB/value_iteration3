//! u64 コストモデル上で動く高速 VI ソルバ群。各ソルバは本家の per-cell 更新
//! `value_iteration_raw` を活性集合に対して呼ぶ。コスト数式は不変なので、到達可能
//! セルの収束値は Reference (全走査) = 本家と bit-exact。
//! 設計: `docs/superpowers/specs/2026-06-09-vi-u64-fast-solvers-design.md`

/// 索引 `it + ix*nt + iy*nt*nx`（本家 `to_index` と整合）のビット集合。
/// フロンティアの活性セル集合を表現する。
pub(crate) struct Bitset3D {
    nx: i32,
    ny: i32,
    nt: i32,
    words: Vec<u64>,
}

impl Bitset3D {
    pub(crate) fn new(nx: i32, ny: i32, nt: i32) -> Self {
        let n = (nx * ny * nt) as usize;
        Bitset3D { nx, ny, nt, words: vec![0u64; n.div_ceil(64)] }
    }
    #[inline]
    fn index(&self, ix: i32, iy: i32, it: i32) -> usize {
        (it + ix * self.nt + iy * self.nt * self.nx) as usize
    }
    pub(crate) fn set(&mut self, ix: i32, iy: i32, it: i32) {
        let i = self.index(ix, iy, it);
        self.words[i / 64] |= 1u64 << (i % 64);
    }
    pub(crate) fn test(&self, ix: i32, iy: i32, it: i32) -> bool {
        let i = self.index(ix, iy, it);
        (self.words[i / 64] >> (i % 64)) & 1 == 1
    }
    pub(crate) fn popcount(&self) -> u64 {
        self.words.iter().map(|w| w.count_ones() as u64).sum()
    }
    pub(crate) fn enumerate(&self) -> impl Iterator<Item = (i32, i32, i32)> + '_ {
        let (nx, ny, nt) = (self.nx, self.ny, self.nt);
        self.words.iter().enumerate().flat_map(move |(wi, &w)| {
            (0..64).filter_map(move |bit| {
                if (w >> bit) & 1 == 1 {
                    let i = (wi * 64 + bit) as i32;
                    let it = i % nt;
                    let ix = (i / nt) % nx;
                    let iy = i / (nt * nx);
                    if iy < ny { Some((ix, iy, it)) } else { None }
                } else {
                    None
                }
            })
        })
    }
    /// 空間 ±dx,±dy（境界クリップ）と θ ±dt（循環 wrap）で膨張した集合を返す。
    pub(crate) fn dilate(&self, dx: i32, dy: i32, dt: i32) -> Bitset3D {
        let mut out = Bitset3D::new(self.nx, self.ny, self.nt);
        for (ix, iy, it) in self.enumerate() {
            for ddx in -dx..=dx {
                let jx = ix + ddx;
                if jx < 0 || jx >= self.nx {
                    continue;
                }
                for ddy in -dy..=dy {
                    let jy = iy + ddy;
                    if jy < 0 || jy >= self.ny {
                        continue;
                    }
                    for ddt in -dt..=dt {
                        let jt = (it + ddt + self.nt) % self.nt;
                        out.set(jx, jy, jt);
                    }
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod bitset_tests {
    use super::Bitset3D;
    #[test]
    fn set_test_popcount_enumerate() {
        let mut b = Bitset3D::new(3, 2, 4); // nx=3, ny=2, nt=4
        assert_eq!(b.popcount(), 0);
        b.set(2, 1, 3);
        b.set(0, 0, 0);
        assert!(b.test(2, 1, 3));
        assert!(b.test(0, 0, 0));
        assert!(!b.test(1, 1, 1));
        assert_eq!(b.popcount(), 2);
        let mut cells: Vec<(i32, i32, i32)> = b.enumerate().collect();
        cells.sort();
        assert_eq!(cells, vec![(0, 0, 0), (2, 1, 3)]);
    }
    #[test]
    fn dilate_spatial_and_theta_wrap() {
        let mut b = Bitset3D::new(5, 5, 4);
        b.set(2, 2, 0);
        let d = b.dilate(1, 1, 1); // ±1 in x,y; ±1 in theta (wrap)
        assert!(d.test(2, 2, 0));
        assert!(d.test(1, 1, 0) && d.test(3, 3, 0));
        assert!(d.test(2, 2, 1) && d.test(2, 2, 3)); // theta wrap 0→{3,1}
        assert!(!d.test(4, 4, 0)); // 距離2は入らない
    }
}
