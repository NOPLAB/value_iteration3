//! 本家 `ValueIterator` 忠実移植 (モデル: 状態・遷移・per-cell Bellman 更新)。
//! 本家の走査実行 (`valueIterationWorker` 単/マルチスレッド経路) は
//! `solvers::original::worker` に住み、`ValueIterator` のメソッドとして生える。

use std::collections::BTreeMap;
use std::f64::consts::PI;

use crate::action::Action;
use crate::msg::{OccupancyGrid, Quaternion};
use crate::params::{MAX_COST, PROB_BASE, PROB_BASE_BIT, RESOLUTION_T_BIT, RESOLUTION_XY_BIT};
use crate::state::State;
use crate::state_transition::StateTransition;
use crate::solvers::original::SweepWorkerStatus;

/// 状態空間へ足す belief 不確かさ次元 `b` のモデル (coastal navigation の augmented MDP)。
///
/// `b` は「自己位置がどれだけ不確かか」の離散レベル (0 = 十分集中 … `nb_levels-1` =
/// ロスト)。実行時の推定値は `crate::belief::Belief::b_hat` が返す。VI 側は
/// 「動けば不確かになり、情報のある場所へ行けば確かになる」という決定的な遷移だけを持つ:
///
/// ```text
/// b' = clamp(b + motion_gain - info[着地セル], 0, nb_levels-1)
/// ```
///
/// `info` は 0/1/2 (0 = 情報なし → b 増、1 = 据置き、2 = 情報リッチ → b 減)。**着地セルだけ**
/// で決まる決定的な写像なので、遷移確率は分割されない (Σprob ≡ PROB_BASE を壊さない)。
///
/// `nb_levels == 1` (既定) では [`BeliefModel::next_ib`] が常に 0 を返し、`states` も
/// 1 層しか作られないので、索引・数式ともに従来の 3 次元 (x,y,θ) VI と**構成的に同一**。
#[derive(Clone, Debug)]
pub struct BeliefModel {
    /// b の離散レベル数。1 = 無効 (既定)。
    pub nb_levels: i32,
    /// `final_state` を許す最大 b。これより不確かな層ではゴールが終端にならない
    /// (= 不確かなままではゴールを「達成」できない)。
    pub ib_goal: i32,
    /// 1 遷移あたりの b 増分 [level]。
    pub motion_gain: i32,
    /// 壁までこの距離以内 → info=2 (b が 1 下がる) [m]。
    pub info_near_m: f64,
    /// 壁までこの距離以内 → info=1 (b 据え置き) [m]。
    pub info_far_m: f64,
    /// `(iy*nx+ix)` → 0/1/2。`set_map_with_occupancy_grid` が構築 (nb_levels>1 のときだけ)。
    pub(crate) info: Vec<u8>,
    pub(crate) nx: i32,
    /// b 1 層ぶんの状態数 `nx*ny*nt`。
    pub(crate) layer: usize,
}

impl Default for BeliefModel {
    fn default() -> Self {
        Self {
            nb_levels: 1,
            ib_goal: 0,
            motion_gain: 1,
            info_near_m: 0.5,
            info_far_m: 2.0,
            info: Vec::new(),
            nx: 0,
            layer: 0,
        }
    }
}

impl BeliefModel {
    /// 着地セル `(ix,iy)` での次の b。`nb_levels <= 1` なら常に 0 (従来経路)。
    /// `(ix,iy)` は呼び出し側で地図内であることを保証すること。
    #[inline]
    pub fn next_ib(&self, ib: i32, ix: i32, iy: i32) -> i32 {
        if self.nb_levels <= 1 {
            return 0;
        }
        (ib + self.motion_gain - self.info[(iy * self.nx + ix) as usize] as i32)
            .clamp(0, self.nb_levels - 1)
    }

    /// 4D 索引 → b 層番号。`nb_levels <= 1` は常に 0。
    #[inline]
    pub(crate) fn ib_of(&self, idx: usize) -> i32 {
        if self.nb_levels <= 1 {
            0
        } else {
            (idx / self.layer) as i32
        }
    }
}

pub struct ValueIterator {
    pub states: Vec<State>,
    pub actions: Vec<Action>,
    pub sweep_orders: Vec<Vec<i32>>,
    pub thread_status: BTreeMap<i32, SweepWorkerStatus>,
    pub status: String,

    pub goal_x: f64,
    pub goal_y: f64,
    pub goal_margin_radius: f64,
    pub goal_t: i32,
    pub goal_margin_theta: i32,
    pub thread_num: i32,

    pub xy_resolution: f64,
    pub t_resolution: f64,
    pub cell_num_x: i32,
    pub cell_num_y: i32,
    pub cell_num_t: i32,
    pub map_origin_x: f64,
    pub map_origin_y: f64,
    pub map_origin_quat: Quaternion,

    /// belief 次元 `b` の設定。**`set_map_with_occupancy_grid` より前**に差し替えること
    /// (`states` の層数と `info` はそこで確定する)。既定 (`nb_levels: 1`) は従来と同一。
    pub belief: BeliefModel,
}

impl ValueIterator {
    /// 本家 `ValueIterator(std::vector<Action> &actions, int thread_num)`。
    pub fn new(actions: Vec<Action>, thread_num: i32) -> Self {
        Self {
            states: Vec::new(),
            actions,
            sweep_orders: Vec::new(),
            thread_status: BTreeMap::new(),
            status: "init".to_string(),
            goal_x: 0.0,
            goal_y: 0.0,
            goal_margin_radius: 0.0,
            goal_t: 0,
            goal_margin_theta: 0,
            thread_num,
            xy_resolution: 0.0,
            t_resolution: 0.0,
            cell_num_x: 0,
            cell_num_y: 0,
            cell_num_t: 0,
            map_origin_x: 0.0,
            map_origin_y: 0.0,
            map_origin_quat: Quaternion::default(),
            belief: BeliefModel::default(),
        }
    }

    /// 本家 `toIndex(ix,iy,it) = it + ix*cell_num_t_ + iy*(cell_num_t_*cell_num_x_)`。
    pub fn to_index(&self, ix: i32, iy: i32, it: i32) -> i32 {
        to_index_raw(ix, iy, it, self.cell_num_x, self.cell_num_t)
    }

    /// belief 次元込みの索引。`b` は**最も遅い**次元なので 3D 索引 + `ib*layer`。
    /// `nb_levels == 1` ではオフセットが literal 0 = [`ValueIterator::to_index`] と同一。
    #[inline]
    pub fn to_index4(&self, ix: i32, iy: i32, it: i32, ib: i32) -> usize {
        to_index_raw(ix, iy, it, self.cell_num_x, self.cell_num_t) as usize
            + ib as usize * self.belief.layer
    }

    /// b の層数 (>= 1)。
    #[inline]
    pub fn belief_levels(&self) -> i32 {
        self.belief.nb_levels.max(1)
    }

    /// 範囲チェック付きの 4D 索引。地図外 / θ 外 / b 層外は `None`。
    #[inline]
    pub(crate) fn idx4_checked(&self, ix: i32, iy: i32, it: i32, ib: i32) -> Option<usize> {
        if !self.in_map_area(ix, iy)
            || it < 0
            || it >= self.cell_num_t
            || ib < 0
            || ib >= self.belief_levels()
        {
            return None;
        }
        Some(self.to_index4(ix, iy, it, ib))
    }

    /// b 1 層ぶんの状態数 `nx*ny*nt` (`states` 長を超えない)。可視化系の writer は
    /// この b=0 スライスだけを見る。
    #[inline]
    fn slice_len(&self) -> usize {
        (self.cell_num_x as usize * self.cell_num_y as usize * self.cell_num_t as usize)
            .min(self.states.len())
    }

    /// 本家 `inMapArea`。
    pub fn in_map_area(&self, ix: i32, iy: i32) -> bool {
        ix >= 0 && ix < self.cell_num_x && iy >= 0 && iy < self.cell_num_y
    }

    /// 本家 `setStateTransition`。θ ごとに 1 スレッドで遷移生成 (書き込み先が
    /// θ 独立なので結果は決定的)。各 action の `state_transitions[it]` を埋める。
    pub(crate) fn set_state_transition(&mut self) {
        let cell_num_t = self.cell_num_t;
        let xy_resolution = self.xy_resolution;
        let t_resolution = self.t_resolution;

        for a in self.actions.iter_mut() {
            a.state_transitions = vec![Vec::new(); cell_num_t as usize];
        }

        let action_params: Vec<(f64, f64)> =
            self.actions.iter().map(|a| (a.delta_fw, a.delta_rot)).collect();

        // per_theta[it][a] を θ 並列で計算。
        let per_theta: Vec<Vec<Vec<StateTransition>>> = std::thread::scope(|scope| {
            let ap = &action_params;
            let handles: Vec<_> = (0..cell_num_t)
                .map(|it| {
                    scope.spawn(move || {
                        ap.iter()
                            .map(|&(fw, rot)| {
                                compute_theta_transitions(fw, rot, it, xy_resolution, t_resolution)
                            })
                            .collect::<Vec<_>>()
                    })
                })
                .collect();
            handles.into_iter().map(|h| h.join().unwrap()).collect()
        });

        for (it, per_action) in per_theta.into_iter().enumerate() {
            for (a, list) in per_action.into_iter().enumerate() {
                self.actions[a].state_transitions[it] = list;
            }
        }
    }

    /// 本家 `setMapWithOccupancyGrid`。
    pub fn set_map_with_occupancy_grid(
        &mut self,
        map: &OccupancyGrid,
        theta_cell_num: i32,
        safety_radius: f64,
        safety_radius_penalty: f64,
        goal_margin_radius: f64,
        goal_margin_theta: i32,
    ) {
        self.cell_num_t = theta_cell_num;
        self.goal_margin_radius = goal_margin_radius;
        self.goal_margin_theta = goal_margin_theta;
        self.cell_num_x = map.width;
        self.cell_num_y = map.height;
        self.xy_resolution = map.resolution;
        // ★整数除算後に f64 化 (本家 `t_resolution_ = 360/cell_num_t_;`)。
        self.t_resolution = (360 / self.cell_num_t) as f64;
        self.map_origin_x = map.origin_x;
        self.map_origin_y = map.origin_y;
        self.map_origin_quat = map.origin_quat.clone();

        self.belief.nx = self.cell_num_x;
        self.belief.layer =
            self.cell_num_x as usize * self.cell_num_y as usize * self.cell_num_t as usize;
        if self.belief.nb_levels > 1 {
            self.build_info_map(map);
        }

        self.set_state(map, safety_radius, safety_radius_penalty);
        self.set_state_transition();
        self.set_sweep_orders();
    }

    /// belief の `info` 場 (0/1/2) を静的地図から起こす。壁までの **city-block 距離**を
    /// チャンファー 2 パスで求め、`info_near_m` / `info_far_m` で 3 値化する。
    /// 地図の外周は障害物として扱わない (= 地図端は情報を与えない)。
    // ponytail: 壁までの L1 距離の粗い代理 (視線も尤度場も見ない)。推定器の尤度場が
    // できたらそれに差し替え。
    fn build_info_map(&mut self, map: &OccupancyGrid) {
        let (nx, ny) = (self.cell_num_x, self.cell_num_y);
        let n = nx as usize * ny as usize;
        let far = i32::MAX / 4;
        let mut d: Vec<i32> = (0..n).map(|i| if map.data[i] != 0 { 0 } else { far }).collect();
        let w = nx as usize;
        for y in 0..ny as usize {
            for x in 0..w {
                let i = y * w + x;
                let mut v = d[i];
                if x > 0 {
                    v = v.min(d[i - 1] + 1);
                }
                if y > 0 {
                    v = v.min(d[i - w] + 1);
                }
                d[i] = v;
            }
        }
        for y in (0..ny as usize).rev() {
            for x in (0..w).rev() {
                let i = y * w + x;
                let mut v = d[i];
                if x + 1 < w {
                    v = v.min(d[i + 1] + 1);
                }
                if y + 1 < ny as usize {
                    v = v.min(d[i + w] + 1);
                }
                d[i] = v;
            }
        }
        let near_cells = self.belief.info_near_m / self.xy_resolution;
        let far_cells = self.belief.info_far_m / self.xy_resolution;
        self.belief.info = d
            .into_iter()
            .map(|dc| {
                let dm = dc as f64;
                if dm <= near_cells {
                    2u8
                } else if dm <= far_cells {
                    1
                } else {
                    0
                }
            })
            .collect();
    }

    /// geometry（cell_num_*/resolution/origin）と遷移テーブルだけを設定し、`states`（O(total)）も
    /// `sweep_orders`（O(total)）も**作らない**。アウトオブコアの `solve_compact_mapped` 用：states を
    /// 持たずに `Geom::build` / `displacement` / `MapSource` を構成できるだけの最小状態を整える。
    pub fn set_map_geometry_no_states(
        &mut self,
        map: &OccupancyGrid,
        theta_cell_num: i32,
        goal_margin_radius: f64,
        goal_margin_theta: i32,
    ) {
        self.cell_num_t = theta_cell_num;
        self.goal_margin_radius = goal_margin_radius;
        self.goal_margin_theta = goal_margin_theta;
        self.cell_num_x = map.width;
        self.cell_num_y = map.height;
        self.xy_resolution = map.resolution;
        self.t_resolution = (360 / self.cell_num_t) as f64;
        self.map_origin_x = map.origin_x;
        self.map_origin_y = map.origin_y;
        self.map_origin_quat = map.origin_quat.clone();
        self.set_state_transition();
        // set_state / set_sweep_orders は呼ばない（compact は states/sweep_orders を使わない）。
    }

    /// 本家 `setState`。
    fn set_state(&mut self, map: &OccupancyGrid, safety_radius: f64, safety_radius_penalty: f64) {
        let margin = (safety_radius / self.xy_resolution).ceil() as i32;
        let (nx, ny, nt) = (self.cell_num_x, self.cell_num_y, self.cell_num_t);
        let n = nx as usize * ny as usize * nt as usize;
        if n == 0 {
            self.states = Vec::new();
            return;
        }
        // 行バンド並列で states を構築。本家の push 順 (y,x,t) を index=((y*nx+x)*nt+t) として
        // そのまま再現するので本家と bit-exact (各 State は map+座標から独立決定。巨大マップでは
        // この per-cell penalty 計算が単一スレッドだと数十秒かかるため並列化する)。
        let per_row = nx as usize * nt as usize; // y 固定 1 行あたりの states 数
        let mut states: Vec<State> = Vec::with_capacity(n);
        let spare = states.spare_capacity_mut();
        let nthr = std::thread::available_parallelism()
            .map(|p| p.get())
            .unwrap_or(1)
            .clamp(1, ny as usize);
        let rows_per = (ny as usize).div_ceil(nthr).max(1);
        std::thread::scope(|s| {
            for (band, chunk) in spare.chunks_mut(rows_per * per_row).enumerate() {
                let y0 = (band * rows_per) as i32;
                s.spawn(move || {
                    let rows = (chunk.len() / per_row) as i32;
                    let mut k = 0usize;
                    for r in 0..rows {
                        let y = y0 + r;
                        for x in 0..nx {
                            for t in 0..nt {
                                chunk[k].write(State::from_occupancy(
                                    x, y, t, map, margin, safety_radius_penalty, nx,
                                ));
                                k += 1;
                            }
                        }
                    }
                });
            }
        });
        // SAFETY: 各バンドが重複なく担当行を埋め、全 n 要素を一度ずつ初期化済み。
        unsafe { states.set_len(n) };
        // belief 層の複製。b 層は map 由来のフィールド (ix/iy/it/free/penalty) が層 0 と
        // 同一なので、層 0 をそのまま複製すれば足りる (層ごとに変わるのは値と final_state
        // だけで、そちらは set_state_values が書く)。nb_levels==1 ならループは回らない。
        for _ in 1..self.belief.nb_levels {
            states.extend_from_within(..n);
        }
        self.states = states;
    }

    /// 本家 `setSweepOrders`。6 種の走査順を生成。既に生成済みなら何もしない。
    /// ★[4]=[0]全体+[1]後半、[5]=[1]前半 というアンバランス/重複を逐語再現。
    pub(crate) fn set_sweep_orders(&mut self) {
        if !self.sweep_orders.is_empty() {
            return;
        }
        let (nx, ny, nt) = (self.cell_num_x, self.cell_num_y, self.cell_num_t);

        // [0]: y, x, t 順
        let mut o0 = Vec::new();
        for y in 0..ny {
            for x in 0..nx {
                for t in 0..nt {
                    o0.push(self.to_index(x, y, t));
                }
            }
        }
        // [1]: x, y, t 順
        let mut o1 = Vec::new();
        for x in 0..nx {
            for y in 0..ny {
                for t in 0..nt {
                    o1.push(self.to_index(x, y, t));
                }
            }
        }
        let o2: Vec<i32> = o0.iter().rev().cloned().collect();
        let o3: Vec<i32> = o1.iter().rev().cloned().collect();
        self.sweep_orders.push(o0); // 0
        self.sweep_orders.push(o1); // 1
        self.sweep_orders.push(o2); // 2
        self.sweep_orders.push(o3); // 3

        // [4],[5]: 本家 `for(i=0;i<2;i++){ push(前半[i]); [4].append(後半[i]); }`
        let half = self.sweep_orders[0].len() / 2;
        // i=0
        let o0_first: Vec<i32> = self.sweep_orders[0][..half].to_vec();
        self.sweep_orders.push(o0_first); // index 4 = [0]前半
        let o0_second: Vec<i32> = self.sweep_orders[0][half..].to_vec();
        self.sweep_orders[4].extend(o0_second); // [4] = [0]全体
        // i=1
        let o1_first: Vec<i32> = self.sweep_orders[1][..half].to_vec();
        self.sweep_orders.push(o1_first); // index 5 = [1]前半
        let o1_second: Vec<i32> = self.sweep_orders[1][half..].to_vec();
        self.sweep_orders[4].extend(o1_second); // [4] = [0]全体 + [1]後半

        // belief 層への拡張: 各走査順を層 0 のまま先頭に置き、層ごとにオフセットを足して
        // 後ろへ継ぎ足す。本家の 6 順 ([4]/[5] のアンバランスを含む) はそのまま保たれ、
        // Reference / sweep_global は 4D 全体を無改造で舐める。nb_levels==1 は no-op。
        if self.belief.nb_levels > 1 {
            let layer = self.belief.layer as i32;
            for order in self.sweep_orders.iter_mut() {
                let base = order.clone();
                for ib in 1..self.belief.nb_levels {
                    order.extend(base.iter().map(|&i| i + ib * layer));
                }
            }
        }
    }

    /// 本家 `actionCost`。公開 3D 契約は据え置き (b=0 スライスで評価)。
    pub fn action_cost(&self, s: &State, a: &Action) -> u64 {
        action_cost_raw(
            &self.states,
            a,
            s,
            0,
            self.cell_num_x,
            self.cell_num_y,
            self.cell_num_t,
            &self.belief,
        )
    }

    /// 本家 `valueIteration` (states[idx] を更新)。`idx` は 4D 索引 (b 層込み)。
    pub fn value_iteration_at(&mut self, idx: usize) -> u64 {
        value_iteration_raw(
            &mut self.states,
            &self.actions,
            idx,
            self.cell_num_x,
            self.cell_num_y,
            self.cell_num_t,
            &self.belief,
        )
    }

    /// 本家 `setGoal`。goal_t を [0,360) に正規化し、final_state を再計算。
    pub fn set_goal(&mut self, goal_x: f64, goal_y: f64, goal_t: i32) {
        let gt = goal_t.rem_euclid(360);
        self.goal_x = goal_x;
        self.goal_y = goal_y;
        self.goal_t = gt;

        self.thread_status.clear();
        self.set_state_values();
        self.status = "calculating".to_string();
    }

    /// 本家 `setStateValues`。距離 + 向き判定で final_state を決め、値を初期化。
    fn set_state_values(&mut self) {
        let (xy_res, ox, oy) = (self.xy_resolution, self.map_origin_x, self.map_origin_y);
        let (gx, gy, gt, gm) = (self.goal_x, self.goal_y, self.goal_t, self.goal_margin_theta);
        let r2 = self.goal_margin_radius * self.goal_margin_radius;
        let t_res = self.t_resolution;

        for s in self.states.iter_mut() {
            // 距離判定
            let x0 = s.ix as f64 * xy_res + ox;
            let y0 = s.iy as f64 * xy_res + oy;
            let r0 = (x0 - gx) * (x0 - gx) + (y0 - gy) * (y0 - gy);
            let x1 = x0 + xy_res;
            let y1 = y0 + xy_res;
            let r1 = (x1 - gx) * (x1 - gx) + (y1 - gy) * (y1 - gy);
            s.final_state = r0 < r2 && r1 < r2 && s.free;

            // 向き判定 (t0/t1 は f64→i32 切り捨て)
            let t0 = (s.it as f64 * t_res) as i32;
            let t1 = ((s.it + 1) as f64 * t_res) as i32;
            let goal_t_2 = if gt > 180 { gt - 360 } else { gt + 360 };
            let ok = (gt - gm <= t0 && t1 <= gt + gm) || (goal_t_2 - gm <= t0 && t1 <= goal_t_2 + gm);
            s.final_state = s.final_state && ok;
        }

        // b 依存のゴール開放 — **これが b の唯一の物理効果**。ib_goal より不確かな層では
        // ゴールが終端にならないので、V は先に info の多いセルを経由する経路へ回る
        // (coastal navigation)。nb_levels==1 では範囲が空でループは回らない。
        // ponytail: b 依存の margin 膨張は未実装 — 要るなら set_state の複製ループで
        // 層ごとに penalty を書き分ける。
        if self.belief.nb_levels > 1 {
            let layer = self.belief.layer;
            for ib in self.belief.ib_goal.clamp(-1, self.belief.nb_levels - 1) + 1
                ..self.belief.nb_levels
            {
                let lo = ib as usize * layer;
                for s in self.states[lo..lo + layer].iter_mut() {
                    s.final_state = false;
                }
            }
        }

        for s in self.states.iter_mut() {
            s.total_cost = if s.final_state { 0 } else { MAX_COST };
            s.local_penalty = 0;
            s.optimal_action = None;
        }
    }

    /// 本家 `valueFunctionWriter`。各 θ 層に `total_cost_/prob_base_`。
    /// ★本家は uint64/uint64 の **整数除算** で小数を切り捨てる
    /// (`map.at(...) = s.total_cost_/prob_base_;`)。`make_value_function_map` 側の
    /// `(double)total_cost_/prob_base_` (浮動小数除算) とは非対称なので注意。
    pub fn value_function_writer(&self) -> GridLayers {
        let (nx, ny, nt) = (self.cell_num_x, self.cell_num_y, self.cell_num_t);
        let mut layers = vec![vec![0f64; (nx * ny) as usize]; nt as usize];
        let n0 = self.slice_len(); // b=0 スライスのみ (nb_levels==1 では states 全体)
        for t in 0..nt {
            let mut i = t;
            while (i as usize) < n0 {
                let s = &self.states[i as usize];
                layers[t as usize][(s.iy * nx + s.ix) as usize] =
                    (s.total_cost / PROB_BASE) as f64;
                i += nt;
            }
        }
        GridLayers { cell_num_x: nx, cell_num_y: ny, cell_num_t: nt, layers }
    }

    /// 本家 `policyWriter`。各 θ 層に optimal_action の id (None は -1)。
    pub fn policy_writer(&self) -> GridLayers {
        let (nx, ny, nt) = (self.cell_num_x, self.cell_num_y, self.cell_num_t);
        let mut layers = vec![vec![0f64; (nx * ny) as usize]; nt as usize];
        let n0 = self.slice_len(); // b=0 スライスのみ
        for t in 0..nt {
            let mut i = t;
            while (i as usize) < n0 {
                let s = &self.states[i as usize];
                let v = match s.optimal_action {
                    None => -1.0,
                    Some(ai) => self.actions[ai].id as f64,
                };
                layers[t as usize][(s.iy * nx + s.ix) as usize] = v;
                i += nt;
            }
        }
        GridLayers { cell_num_x: nx, cell_num_y: ny, cell_num_t: nt, layers }
    }

    /// 本家 `makeValueFunctionMap`。i8 への push ラップ (250→-6, 255→-1) を再現。
    pub fn make_value_function_map(
        &self,
        threshold: i32,
        _x: f64,
        _y: f64,
        yaw_rad: f64,
    ) -> OccupancyGrid {
        let (nx, ny) = (self.cell_num_x, self.cell_num_y);
        let it = ((((yaw_rad / PI * 180.0) as i32 + 360 * 100) % 360) as f64 / self.t_resolution)
            .floor() as i32;
        let mut data: Vec<i8> = Vec::with_capacity((nx * ny) as usize);
        for y in 0..ny {
            for x in 0..nx {
                let index = self.to_index(x, y, it) as usize; // b=0 スライス
                let cost = self.states[index].total_cost as f64 / PROB_BASE as f64;
                let val: i32 = if cost < threshold as f64 {
                    (cost / threshold as f64 * 250.0) as i32
                } else if self.states[index].free {
                    250
                } else {
                    255
                };
                data.push(val as u8 as i8); // ★i8 ラップ
            }
        }
        OccupancyGrid {
            width: nx,
            height: ny,
            resolution: self.xy_resolution,
            origin_x: self.map_origin_x,
            origin_y: self.map_origin_y,
            origin_quat: self.map_origin_quat.clone(),
            data,
        }
    }

    pub fn set_cancel(&mut self) {
        self.status = "canceled".to_string();
    }
    pub fn end_of_trial(&self) -> bool {
        self.status == "canceled" || self.status == "goal"
    }
    pub fn set_calculated(&mut self) {
        if self.status != "canceled" {
            self.status = "calculated".to_string();
        }
    }
    pub fn is_calculated(&self) -> bool {
        self.status == "calculated"
    }
}

// ── コア free 関数 (単スレッド経路とマルチスレッド経路で共有) ──

#[inline]
pub(crate) fn to_index_raw(ix: i32, iy: i32, it: i32, cell_num_x: i32, cell_num_t: i32) -> i32 {
    it + ix * cell_num_t + iy * (cell_num_t * cell_num_x)
}

/// 本家 `cellDelta`。`it` は絶対インデックス (負正規化しない)。
pub(crate) fn cell_delta(
    x: f64,
    y: f64,
    t: f64,
    xy_resolution: f64,
    t_resolution: f64,
) -> (i32, i32, i32) {
    let mut ix = (x.abs() / xy_resolution).floor() as i32;
    if x < 0.0 {
        ix = -ix - 1;
    }
    let mut iy = (y.abs() / xy_resolution).floor() as i32;
    if y < 0.0 {
        iy = -iy - 1;
    }
    let it = (t / t_resolution).floor() as i32;
    (ix, iy, it)
}

/// 本家 `noNoiseStateTransition`。`to_t` は負方向しか正規化しない (>=360 は残す)。
pub(crate) fn no_noise_state_transition(
    delta_fw: f64,
    delta_rot: f64,
    from_x: f64,
    from_y: f64,
    from_t: f64,
) -> (f64, f64, f64) {
    let ang = from_t / 180.0 * PI;
    let to_x = from_x + delta_fw * ang.cos();
    let to_y = from_y + delta_fw * ang.sin();
    let mut to_t = from_t + delta_rot;
    while to_t < 0.0 {
        to_t += 360.0;
    }
    (to_x, to_y, to_t)
}

/// 本家 `setStateTransitionWorkerSub` の 1 (action, theta) 分。
/// サブセルサンプリングで遷移先バケットを集計する。`dit` は絶対 θ。
pub(crate) fn compute_theta_transitions(
    delta_fw: f64,
    delta_rot: f64,
    it: i32,
    xy_resolution: f64,
    t_resolution: f64,
) -> Vec<StateTransition> {
    let theta_origin = it as f64 * t_resolution;
    let xy_sample_num = 1i32 << RESOLUTION_XY_BIT; // 64
    let t_sample_num = 1i32 << RESOLUTION_T_BIT; // 64
    let xy_step = xy_resolution / xy_sample_num as f64;
    let t_step = t_resolution / t_sample_num as f64;

    let mut out: Vec<StateTransition> = Vec::new();

    // 本家 `for(double o=0.5*step; o<limit; o+=step)` の f64 累積を忠実再現。
    let mut oy = 0.5 * xy_step;
    while oy < xy_resolution {
        let mut ox = 0.5 * xy_step;
        while ox < xy_resolution {
            let mut ot = 0.5 * t_step;
            while ot < t_resolution {
                let (dx, dy, dt) =
                    no_noise_state_transition(delta_fw, delta_rot, ox, oy, ot + theta_origin);
                let (dix, diy, dit) = cell_delta(dx, dy, dt, xy_resolution, t_resolution);

                let mut exist = false;
                for s in out.iter_mut() {
                    if s.dix == dix && s.diy == diy && s.dit == dit {
                        s.prob += 1;
                        exist = true;
                        break;
                    }
                }
                if !exist {
                    out.push(StateTransition::new(dix, diy, dit, 1));
                }
                ot += t_step;
            }
            ox += xy_step;
        }
        oy += xy_step;
    }
    out
}

/// 本家 `actionCost`。★u64 オーバーフロー折り返しを `wrapping_*` で再現。
/// `dit` は絶対 θ なので `(dit + nt) % nt` で wrap (s.it は足さない)。
///
/// `ib` は評価対象状態の belief レベル (`s` には持たせない — b は最も遅い次元なので
/// 索引から導ける)。着地セルごとに `belief.next_ib` で b' を決めて 4D 索引を引く。
/// `nb_levels == 1` では `next_ib` が 0 を返すのでオフセットは literal 0 = 従来と同一。
#[allow(clippy::too_many_arguments)]
pub(crate) fn action_cost_raw(
    states: &[State],
    a: &Action,
    s: &State,
    ib: i32,
    cell_num_x: i32,
    cell_num_y: i32,
    cell_num_t: i32,
    belief: &BeliefModel,
) -> u64 {
    let mut cost: u64 = 0;
    for tran in &a.state_transitions[s.it as usize] {
        let ix = s.ix + tran.dix;
        if ix < 0 || ix >= cell_num_x {
            return MAX_COST;
        }
        let iy = s.iy + tran.diy;
        if iy < 0 || iy >= cell_num_y {
            return MAX_COST;
        }
        let it = (tran.dit + cell_num_t) % cell_num_t;
        let ib2 = belief.next_ib(ib, ix, iy);
        let after = &states[to_index_raw(ix, iy, it, cell_num_x, cell_num_t) as usize
            + ib2 as usize * belief.layer];
        if !after.free {
            return MAX_COST;
        }
        cost = cost.wrapping_add(
            after
                .total_cost
                .wrapping_add(after.penalty)
                .wrapping_add(after.local_penalty)
                .wrapping_mul(tran.prob as u64),
        );
    }
    cost >> PROB_BASE_BIT
}

/// 本家 `valueIteration`。free でない/final_state なら 0 を返し更新しない。
/// `final_state`/非 `free` セルは `None`。それ以外は **書き込まずに** min over アクションの
/// `(min_cost, optimal_action)` を返す。u64 高速ソルバの近似版（Tau の非書込閾値判定等）で使う。
#[allow(clippy::too_many_arguments)]
pub(crate) fn min_action_cost(
    states: &[State],
    actions: &[Action],
    idx: usize,
    cell_num_x: i32,
    cell_num_y: i32,
    cell_num_t: i32,
    belief: &BeliefModel,
) -> Option<(u64, Option<usize>)> {
    if !states[idx].free || states[idx].final_state {
        return None;
    }
    let ib = belief.ib_of(idx);
    let mut min_cost: u64 = MAX_COST;
    let mut min_action: Option<usize> = None;
    let s = &states[idx];
    for (ai, a) in actions.iter().enumerate() {
        let c = action_cost_raw(states, a, s, ib, cell_num_x, cell_num_y, cell_num_t, belief);
        if c < min_cost {
            min_cost = c;
            min_action = Some(ai);
        }
    }
    Some((min_cost, min_action))
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn value_iteration_raw(
    states: &mut [State],
    actions: &[Action],
    idx: usize,
    cell_num_x: i32,
    cell_num_y: i32,
    cell_num_t: i32,
    belief: &BeliefModel,
) -> u64 {
    let Some((min_cost, min_action)) =
        min_action_cost(states, actions, idx, cell_num_x, cell_num_y, cell_num_t, belief)
    else {
        return 0;
    };
    let old = states[idx].total_cost;
    let delta = (min_cost as i64) - (old as i64);
    states[idx].total_cost = min_cost;
    states[idx].optimal_action = min_action;
    delta.unsigned_abs()
}

/// 本家 `valueFunctionWriter` / `policyWriter` 相当のプレーンデータ。
/// `layers[t]` は長さ `cell_num_x*cell_num_y`、索引 `iy*cell_num_x + ix`。
pub struct GridLayers {
    pub cell_num_x: i32,
    pub cell_num_y: i32,
    pub cell_num_t: i32,
    pub layers: Vec<Vec<f64>>,
}

#[cfg(test)]
#[path = "value_iterator_tests.rs"]
mod tests;
