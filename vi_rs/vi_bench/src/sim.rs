//! follow_ctrl_bench / viola_bench が共用する閉ループシミュレーション部品。
//! 地図上のスタート乱択・クリアランス距離場・決定的乱数・greedy 判断器など、
//! 「解いた場の上でロボットを走らせる」ベンチの足回りだけを置く (指標の集計や
//! 走行ループ自体は各 bin 固有)。

use crate::params::canonical_actions;
use vi_lib::planner::PolicyView;
use vi_lib::{Action, ValueIterator};

/// bench_map と同じ「最寄りの free セルへスナップ」。
pub fn snap_to_free(occ: &[i8], w: i32, h: i32, gx: i32, gy: i32, max_r: i32) -> Option<(i32, i32)> {
    let at = |x: i32, y: i32| (y * w + x) as usize;
    if gx >= 0 && gx < w && gy >= 0 && gy < h && occ[at(gx, gy)] == 0 {
        return Some((gx, gy));
    }
    for r in 1..=max_r {
        for dy in -r..=r {
            for dx in -r..=r {
                if dx.abs() != r && dy.abs() != r {
                    continue;
                }
                let (nx, ny) = (gx + dx, gy + dy);
                if nx < 0 || ny < 0 || nx >= w || ny >= h {
                    continue;
                }
                if occ[at(nx, ny)] == 0 {
                    return Some((nx, ny));
                }
            }
        }
    }
    None
}

/// 2 パス chamfer (3-4) 距離変換。障害物セルからの近似ユークリッド距離 (単位 res/3)。
pub fn chamfer_dist(occ: &[i8], w: i32, h: i32) -> Vec<u32> {
    const INF: u32 = u32::MAX / 2;
    let mut d: Vec<u32> = occ.iter().map(|&c| if c != 0 { 0 } else { INF }).collect();
    let at = |x: i32, y: i32| (y * w + x) as usize;
    for y in 0..h {
        for x in 0..w {
            let mut v = d[at(x, y)];
            if x > 0 {
                v = v.min(d[at(x - 1, y)] + 3);
            }
            if y > 0 {
                v = v.min(d[at(x, y - 1)] + 3);
                if x > 0 {
                    v = v.min(d[at(x - 1, y - 1)] + 4);
                }
                if x < w - 1 {
                    v = v.min(d[at(x + 1, y - 1)] + 4);
                }
            }
            d[at(x, y)] = v;
        }
    }
    for y in (0..h).rev() {
        for x in (0..w).rev() {
            let mut v = d[at(x, y)];
            if x < w - 1 {
                v = v.min(d[at(x + 1, y)] + 3);
            }
            if y < h - 1 {
                v = v.min(d[at(x, y + 1)] + 3);
                if x < w - 1 {
                    v = v.min(d[at(x + 1, y + 1)] + 4);
                }
                if x > 0 {
                    v = v.min(d[at(x - 1, y + 1)] + 4);
                }
            }
            d[at(x, y)] = v;
        }
    }
    d
}

/// xorshift64* (決定的乱択用)。
pub struct Rng(pub u64);
impl Rng {
    pub fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    pub fn below(&mut self, n: u64) -> u64 {
        self.next() % n.max(1)
    }
    /// 一様 (0,1)。
    pub fn unit(&mut self) -> f64 {
        ((self.next() >> 11) as f64 + 1.0) / (1u64 << 53) as f64
    }
    /// 標準正規 (Box–Muller の片側)。
    pub fn gauss(&mut self) -> f64 {
        let (u1, u2) = (self.unit(), self.unit());
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

pub fn scaled_actions(scale: f64) -> Vec<Action> {
    canonical_actions()
        .into_iter()
        .enumerate()
        .map(|(i, a)| Action::new(&a.name, a.delta_fw * scale, a.delta_rot, i as i32))
        .collect()
}

/// vi_planner `PlannerCore::decide` 相当 (本家 `posToAction` + 近傍借用)。
pub enum GreedyOut {
    Goal,
    Act(f64, f64),
    NoAction,
}

pub fn greedy_decide(vi: &ValueIterator, ix: i32, iy: i32, it: i32, tol: i32) -> GreedyOut {
    if vi.is_final(ix, iy, it) {
        return GreedyOut::Goal;
    }
    if let Some(ai) = vi.action_index(ix, iy, it) {
        let (fw, rot) = vi.action_delta(ai);
        return GreedyOut::Act(fw, rot);
    }
    let mut best: Option<(i64, GreedyOut)> = None;
    for dy in -tol..=tol {
        for dx in -tol..=tol {
            if dx == 0 && dy == 0 {
                continue;
            }
            let (nx, ny) = (ix + dx, iy + dy);
            let cand = if vi.is_final(nx, ny, it) {
                GreedyOut::Goal
            } else if let Some(ai) = vi.action_index(nx, ny, it) {
                let (fw, rot) = vi.action_delta(ai);
                GreedyOut::Act(fw, rot)
            } else {
                continue;
            };
            let d2 = (dx as i64) * (dx as i64) + (dy as i64) * (dy as i64);
            if best.as_ref().map(|(bd, _)| d2 < *bd).unwrap_or(true) {
                best = Some((d2, cand));
            }
        }
    }
    best.map(|(_, c)| c).unwrap_or(GreedyOut::NoAction)
}

pub fn mean(vals: impl Iterator<Item = f64>) -> f64 {
    let v: Vec<f64> = vals.collect();
    if v.is_empty() {
        f64::NAN
    } else {
        v.iter().sum::<f64>() / v.len() as f64
    }
}

/// `in_map_area` + θ 範囲のガード (走行ループの脱出判定用)。
pub fn in_field(vi: &ValueIterator, ix: i32, iy: i32, it: i32) -> bool {
    PolicyView::in_map_area(vi, ix, iy) && it >= 0 && it < vi.cell_num_t
}
