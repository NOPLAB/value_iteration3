//! u64 (vi_reference) ベンチ用フィクスチャ。合成 `OccupancyGrid` から、本家忠実な
//! セット済み `ValueIterator` を組み立てる。u16 時代の `vi_fixtures` ベース
//! `VIContext` ビルダ (`build_context`) を置き換える。
//!
//! ベンチ入力は本家モデル（PROB_BASE 固定小数）なので、`vi_reference::solvers::solve`
//! の全ソルバが Reference（=本家）と bit-exact になる。

use vi_reference::{Action, OccupancyGrid, Quaternion, ValueIterator};

/// ベンチ共通パラメータ（vi_compare / 本家 launch と整合）。
const RES: f64 = 0.05;
const THETA_CELL_NUM: i32 = 60;
const SAFETY_RADIUS: f64 = 0.2;
const SAFETY_PENALTY: f64 = 30.0;
/// 小さめのゴール半径（2 セル）。大きいと小マップでほぼ全セルが goal=cost0 になり
/// 伝播仕事が消えるため、ベンチとして意味を持たせるため控えめにする。
const GOAL_MARGIN_RADIUS: f64 = 0.10;
const GOAL_MARGIN_THETA: i32 = 15;

/// 本家 launch と ID 順まで一致する正典 6 アクション。
pub fn canonical_actions() -> Vec<Action> {
    vec![
        Action::new("forward", 0.3, 0.0, 0),
        Action::new("back", -0.2, 0.0, 1),
        Action::new("right", 0.0, -20.0, 2),
        Action::new("rightfw", 0.2, -20.0, 3),
        Action::new("left", 0.0, 20.0, 4),
        Action::new("leftfw", 0.2, 20.0, 5),
    ]
}

/// 合成マップ種別。u16 ベンチの `vi_fixtures::MapType` 相当を vi_reference 入力
/// （occupancy i8）として再現する。
#[derive(Clone, Copy, Debug)]
pub enum BenchMap {
    Empty,
    Obstacle,
    Sentinel,
    Random { density: f64, seed: u64 },
}

impl BenchMap {
    /// CLI ラベルから種別を引く（`bench_summary --types` 用）。
    pub fn from_name(s: &str) -> Option<Self> {
        Some(match s {
            "empty" => BenchMap::Empty,
            "obstacle" => BenchMap::Obstacle,
            "sentinel" => BenchMap::Sentinel,
            "random" => BenchMap::Random { density: 0.15, seed: 42 },
            _ => return None,
        })
    }
}

/// 決定的 xorshift64（`rand` 依存を避ける）。
fn xorshift64(state: &mut u64) -> u64 {
    let mut x = *state;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *state = x;
    x
}

/// row-major `(h*w)` の occupancy i8（0=free / 100=obstacle）を生成。ゴールセル
/// （中心）は常に free に保つ。`ValueIterator` は ROS と同じく `data[x + w*y]`
/// で読む（y=0 が world 原点側）ので、合成マップもその規約で並べる。
fn gen_occupancy(w: u32, h: u32, map: BenchMap) -> Vec<i8> {
    let (w, h) = (w as usize, h as usize);
    let mut occ = vec![0i8; w * h];
    let (gx, gy) = (w / 2, h / 2); // ゴールセル（中心）
    let at = |x: usize, y: usize| y * w + x;
    match map {
        BenchMap::Empty => {}
        BenchMap::Obstacle => {
            // 中央の縦壁（ゴール行に隙間）。左右はゴール（隙間）経由でのみ連結。
            let wallx = w / 2;
            for y in 0..h {
                if y == gy {
                    continue;
                }
                occ[at(wallx, y)] = 100;
            }
        }
        BenchMap::Sentinel => {
            // ゴールを左・右・下の3方向で囲む（goal-neighbor sentinel 経路を踏む）。
            if gx >= 1 {
                occ[at(gx - 1, gy)] = 100;
            }
            if gx + 1 < w {
                occ[at(gx + 1, gy)] = 100;
            }
            if gy >= 1 {
                occ[at(gx, gy - 1)] = 100;
            }
        }
        BenchMap::Random { density, seed } => {
            let mut st = seed | 1; // 0 種を避ける
            let thresh = (density * u64::MAX as f64) as u64;
            for cell in occ.iter_mut() {
                if xorshift64(&mut st) < thresh {
                    *cell = 100;
                }
            }
        }
    }
    occ[at(gx, gy)] = 0; // ゴールは必ず free
    occ
}

/// `size×size` の合成マップ + 中心ゴールでセット済みの `ValueIterator` を構築。
/// `ValueIterator` は `Clone` でないので、各ベンチ反復はこれを呼び直して初期化する。
pub fn build_vi(size: u32, map: BenchMap) -> ValueIterator {
    let data = gen_occupancy(size, size, map);
    let grid = OccupancyGrid {
        width: size as i32,
        height: size as i32,
        resolution: RES,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data,
    };
    let mut vi = ValueIterator::new(canonical_actions(), 1);
    vi.set_map_with_occupancy_grid(
        &grid,
        THETA_CELL_NUM,
        SAFETY_RADIUS,
        SAFETY_PENALTY,
        GOAL_MARGIN_RADIUS,
        GOAL_MARGIN_THETA,
    );
    // 中心ゴール（world 座標）。set_goal の goal_t は度（本家 executeVi 準拠）。
    let g_world = size as f64 * RES / 2.0;
    vi.set_goal(g_world, g_world, 0);
    vi
}

#[cfg(test)]
mod tests {
    use super::*;
    use vi_reference::params::PROB_BASE;

    const REACH: u64 = 1_000_000u64 * PROB_BASE;

    #[test]
    fn build_vi_has_requested_dimensions() {
        let vi = build_vi(8, BenchMap::Empty);
        assert_eq!(vi.cell_num_x, 8);
        assert_eq!(vi.cell_num_y, 8);
        assert_eq!(vi.cell_num_t, THETA_CELL_NUM);
        assert_eq!(vi.states.len(), 8 * 8 * THETA_CELL_NUM as usize);
    }

    #[test]
    fn goal_cells_exist_for_each_map_type() {
        for map in [
            BenchMap::Empty,
            BenchMap::Obstacle,
            BenchMap::Sentinel,
            BenchMap::Random { density: 0.15, seed: 7 },
        ] {
            let vi = build_vi(8, map);
            let n_goal = vi.states.iter().filter(|s| s.total_cost < REACH).count();
            assert!(n_goal > 0, "goal セルが存在するはず ({map:?})");
        }
    }

    #[test]
    fn from_name_round_trips() {
        assert!(matches!(BenchMap::from_name("empty"), Some(BenchMap::Empty)));
        assert!(matches!(BenchMap::from_name("obstacle"), Some(BenchMap::Obstacle)));
        assert!(matches!(BenchMap::from_name("sentinel"), Some(BenchMap::Sentinel)));
        assert!(matches!(BenchMap::from_name("random"), Some(BenchMap::Random { .. })));
        assert!(BenchMap::from_name("nope").is_none());
    }
}
