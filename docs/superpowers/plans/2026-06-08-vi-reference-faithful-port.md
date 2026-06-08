# 本家 value_iteration 忠実移植 reference 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 本家 ROS1 `value_iteration` の `ValueIterator` / `ValueIteratorLocal` を、型・アルゴリズム・固有バグまで忠実再現した新クレート `vi_rs/vi_reference/` を実装する。

**Architecture:** `vi_core` 非依存・`std` のみの独立クレート。コスト `u64` / 座標・prob `i32` / 解像度 `f64`。コア計算 (`action_cost_raw` / `value_iteration_raw` / `to_index_raw`) をフリー関数化して単スレッド経路とマルチスレッド経路で共有。マルチスレッドは `*mut State` スライス共有 + `unsafe` で本家のデータ競合を再現し、`thread_num=1` を決定的テスト基準とする。

**Tech Stack:** Rust 2021 (rust-version 1.75), std::thread::scope, ワークスペースメンバ追加のみ。

**仕様:** `docs/superpowers/specs/2026-06-08-vi-reference-faithful-port-design.md`

---

## ファイル構成

```
vi_rs/vi_reference/
├─ Cargo.toml
└─ src/
   ├─ lib.rs              # モジュール宣言 + re-export
   ├─ params.rs           # 定数 (PROB_BASE 等)
   ├─ msg.rs              # OccupancyGrid / Quaternion / LaserScan
   ├─ state_transition.rs # StateTransition
   ├─ action.rs           # Action
   ├─ sweep_status.rs     # SweepWorkerStatus
   ├─ state.rs            # State + 2 コンストラクタ (margin バグ)
   ├─ value_iterator.rs   # ValueIterator フルパイプライン + コア free fn
   └─ local.rs            # ValueIteratorLocal
```

各タスクは `cd vi_rs && cargo test -p vi_reference` で検証する。コミットはローカルブランチ前提 (作業ブランチ `feat/vi-reference-faithful-port` で実行)。

---

## Task 1: クレート雛形 + 定数 + ワークスペース登録

**Files:**
- Create: `vi_rs/vi_reference/Cargo.toml`
- Create: `vi_rs/vi_reference/src/lib.rs`
- Create: `vi_rs/vi_reference/src/params.rs`
- Modify: `vi_rs/Cargo.toml` (members に追加)

- [ ] **Step 1: `Cargo.toml` を作成**

`vi_rs/vi_reference/Cargo.toml`:
```toml
[package]
name = "vi_reference"
version = "0.1.0"
edition.workspace = true
rust-version.workspace = true

[dependencies]
```

- [ ] **Step 2: `params.rs` を作成**

`vi_rs/vi_reference/src/params.rs`:
```rust
//! 本家 `ValueIterator.h` 末尾の静的定数を忠実再現。
//!
//! ```cpp
//! const unsigned char resolution_xy_bit_ = 6;
//! const unsigned char resolution_t_bit_  = 6;
//! const unsigned char prob_base_bit_ = resolution_xy_bit_*2 + resolution_t_bit_; // 18
//! const uint64_t prob_base_ = 1<<prob_base_bit_;            // 262144
//! const uint64_t max_cost_  = 1000000000*prob_base_;        // 262_144_000_000_000
//! ```

pub const RESOLUTION_XY_BIT: u32 = 6;
pub const RESOLUTION_T_BIT: u32 = 6;
pub const PROB_BASE_BIT: u32 = RESOLUTION_XY_BIT * 2 + RESOLUTION_T_BIT; // 18
pub const PROB_BASE: u64 = 1u64 << PROB_BASE_BIT; // 262144
pub const MAX_COST: u64 = 1_000_000_000u64 * PROB_BASE; // 262_144_000_000_000

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constants_match_original() {
        assert_eq!(PROB_BASE_BIT, 18);
        assert_eq!(PROB_BASE, 262_144);
        assert_eq!(MAX_COST, 262_144_000_000_000);
    }
}
```

- [ ] **Step 3: `lib.rs` を作成 (params のみ宣言)**

`vi_rs/vi_reference/src/lib.rs`:
```rust
//! 本家 ROS1 `value_iteration` パッケージ (`ValueIterator` / `ValueIteratorLocal`) の
//! Rust 忠実移植。型・アルゴリズム・固有バグまで一致させることを目的とする。
//! 設計: `docs/superpowers/specs/2026-06-08-vi-reference-faithful-port-design.md`

pub mod params;
```

- [ ] **Step 4: ワークスペースに登録**

`vi_rs/Cargo.toml` の members を次のように変更:
```toml
members = ["vi_core", "vi_algorithm", "vi_fixtures", "vi_bench", "vi_reference"]
```

- [ ] **Step 5: ビルド & テスト**

Run: `cd vi_rs && cargo test -p vi_reference`
Expected: PASS (`constants_match_original`)

- [ ] **Step 6: Commit**

```bash
git add vi_rs/Cargo.toml vi_rs/vi_reference
git commit -m "feat(vi_reference): scaffold crate with faithful constants"
```

---

## Task 2: 入力メッセージ型 (`msg.rs`)

**Files:**
- Create: `vi_rs/vi_reference/src/msg.rs`
- Modify: `vi_rs/vi_reference/src/lib.rs`

- [ ] **Step 1: `msg.rs` を作成**

`vi_rs/vi_reference/src/msg.rs`:
```rust
//! 本家が参照する ROS メッセージのフィールドのみを持つ最小代替型。

/// `geometry_msgs::Quaternion` 相当 (計算には使わず echo されるだけ)。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

/// `nav_msgs::OccupancyGrid` の使用フィールドのみ。
/// `data` は ROS の int8 (0=free、それ以外=占有/unknown)。
#[derive(Clone, Debug, Default)]
pub struct OccupancyGrid {
    pub width: i32,
    pub height: i32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub origin_quat: Quaternion,
    pub data: Vec<i8>,
}

/// `sensor_msgs::LaserScan` の使用フィールドのみ。
#[derive(Clone, Debug, Default)]
pub struct LaserScan {
    pub angle_min: f64,
    pub angle_increment: f64,
    pub ranges: Vec<f64>,
}
```

- [ ] **Step 2: `lib.rs` に追加**

`vi_rs/vi_reference/src/lib.rs` の `pub mod params;` の後に追記:
```rust
pub mod msg;

pub use msg::{LaserScan, OccupancyGrid, Quaternion};
```

- [ ] **Step 3: ビルド**

Run: `cd vi_rs && cargo build -p vi_reference`
Expected: PASS (warning 無し)

- [ ] **Step 4: Commit**

```bash
git add vi_rs/vi_reference/src/msg.rs vi_rs/vi_reference/src/lib.rs
git commit -m "feat(vi_reference): add minimal ROS message types"
```

---

## Task 3: `StateTransition` (`state_transition.rs`)

**Files:**
- Create: `vi_rs/vi_reference/src/state_transition.rs`
- Modify: `vi_rs/vi_reference/src/lib.rs`

- [ ] **Step 1: 失敗するテストを書く**

`vi_rs/vi_reference/src/state_transition.rs`:
```rust
//! 本家 `StateTransition` 忠実移植。

/// 1 つの遷移先。`dix`/`diy` は変位 (delta)、`dit` は **絶対 θ インデックス**。
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StateTransition {
    pub dix: i32,
    pub diy: i32,
    pub dit: i32,
    pub prob: i32,
}

impl StateTransition {
    pub fn new(dix: i32, diy: i32, dit: i32, prob: i32) -> Self {
        Self { dix, diy, dit, prob }
    }

    /// 本家 `StateTransition::to_string`。
    #[allow(clippy::inherent_to_string)]
    pub fn to_string(&self) -> String {
        format!(
            "dix:{} diy:{} dit:{} prob:{}",
            self.dix, self.diy, self.dit, self.prob
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn to_string_matches_original_format() {
        let st = StateTransition::new(1, -2, 3, 4);
        assert_eq!(st.to_string(), "dix:1 diy:-2 dit:3 prob:4");
    }
}
```

- [ ] **Step 2: `lib.rs` に追加**

`vi_rs/vi_reference/src/lib.rs` に追記:
```rust
pub mod state_transition;

pub use state_transition::StateTransition;
```

- [ ] **Step 3: テスト実行**

Run: `cd vi_rs && cargo test -p vi_reference state_transition`
Expected: PASS

- [ ] **Step 4: Commit**

```bash
git add vi_rs/vi_reference/src/state_transition.rs vi_rs/vi_reference/src/lib.rs
git commit -m "feat(vi_reference): add StateTransition"
```

---

## Task 4: `Action` (`action.rs`)

**Files:**
- Create: `vi_rs/vi_reference/src/action.rs`
- Modify: `vi_rs/vi_reference/src/lib.rs`

- [ ] **Step 1: `action.rs` を作成**

`vi_rs/vi_reference/src/action.rs`:
```rust
//! 本家 `Action` 忠実移植。

use crate::state_transition::StateTransition;

/// 行動 1 つ。`state_transitions[theta]` が θ ごとの遷移先リスト。
#[derive(Clone, Debug)]
pub struct Action {
    pub name: String,
    pub delta_fw: f64,  // _delta_fw [m]
    pub delta_rot: f64, // _delta_rot [deg]
    pub id: i32,        // id_
    pub state_transitions: Vec<Vec<StateTransition>>,
}

impl Action {
    /// 本家 `Action(std::string name, double fw, double rot, int id)`。
    pub fn new(name: impl Into<String>, fw: f64, rot: f64, id: i32) -> Self {
        Self {
            name: name.into(),
            delta_fw: fw,
            delta_rot: rot,
            id,
            state_transitions: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_sets_fields() {
        let a = Action::new("forward", 0.3, 0.0, 0);
        assert_eq!(a.name, "forward");
        assert_eq!(a.delta_fw, 0.3);
        assert_eq!(a.delta_rot, 0.0);
        assert_eq!(a.id, 0);
        assert!(a.state_transitions.is_empty());
    }
}
```

- [ ] **Step 2: `lib.rs` に追加**

```rust
pub mod action;

pub use action::Action;
```

- [ ] **Step 3: テスト実行**

Run: `cd vi_rs && cargo test -p vi_reference action`
Expected: PASS

- [ ] **Step 4: Commit**

```bash
git add vi_rs/vi_reference/src/action.rs vi_rs/vi_reference/src/lib.rs
git commit -m "feat(vi_reference): add Action"
```

---

## Task 5: `SweepWorkerStatus` (`sweep_status.rs`)

**Files:**
- Create: `vi_rs/vi_reference/src/sweep_status.rs`
- Modify: `vi_rs/vi_reference/src/lib.rs`

- [ ] **Step 1: `sweep_status.rs` を作成**

`vi_rs/vi_reference/src/sweep_status.rs`:
```rust
//! 本家 `SweepWorkerStatus` 忠実移植。
//! 本家コンストラクタは `_finished=false; _sweep_step=0; _delta=max_cost_`。

use crate::params::MAX_COST;

#[derive(Clone, Debug, PartialEq)]
pub struct SweepWorkerStatus {
    pub finished: bool,
    pub sweep_step: i32,
    pub delta: f64,
}

impl Default for SweepWorkerStatus {
    fn default() -> Self {
        Self {
            finished: false,
            sweep_step: 0,
            delta: MAX_COST as f64,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_matches_original() {
        let s = SweepWorkerStatus::default();
        assert!(!s.finished);
        assert_eq!(s.sweep_step, 0);
        assert_eq!(s.delta, MAX_COST as f64);
    }
}
```

- [ ] **Step 2: `lib.rs` に追加**

```rust
pub mod sweep_status;

pub use sweep_status::SweepWorkerStatus;
```

- [ ] **Step 3: テスト実行**

Run: `cd vi_rs && cargo test -p vi_reference sweep_status`
Expected: PASS

- [ ] **Step 4: Commit**

```bash
git add vi_rs/vi_reference/src/sweep_status.rs vi_rs/vi_reference/src/lib.rs
git commit -m "feat(vi_reference): add SweepWorkerStatus"
```

---

## Task 6: `State` + 2 コンストラクタ (margin バグ込み) (`state.rs`)

**Files:**
- Create: `vi_rs/vi_reference/src/state.rs`
- Modify: `vi_rs/vi_reference/src/lib.rs`

- [ ] **Step 1: 失敗するテストを書く (margin 行跨ぎバグの再現確認込み)**

`vi_rs/vi_reference/src/state.rs`:
```rust
//! 本家 `State` 忠実移植。コンストラクタ 2 種。
//! occupancy 版の margin penalty ループは線形 `pos` 境界のみで列範囲を見ない
//! **固有バグ**を保持する。

use crate::msg::OccupancyGrid;
use crate::params::{MAX_COST, PROB_BASE, PROB_BASE_BIT};

#[derive(Clone, Debug)]
pub struct State {
    pub total_cost: u64,
    pub penalty: u64,
    pub local_penalty: u64,
    pub ix: i32,
    pub iy: i32,
    pub it: i32,
    pub free: bool,
    pub final_state: bool,
    /// 本家 `Action *optimal_action_` → `actions` ベクタへの索引。
    pub optimal_action: Option<usize>,
}

impl State {
    /// 本家 `State(int x, int y, int theta, const nav_msgs::OccupancyGrid &map,
    ///            int margin, double margin_penalty, int x_num)`。
    pub fn from_occupancy(
        x: i32,
        y: i32,
        theta: i32,
        map: &OccupancyGrid,
        margin: i32,
        margin_penalty: f64,
        x_num: i32,
    ) -> Self {
        // 本家: margin_penalty>1e10 で ROS_ERROR を出すだけ (計算続行)。ここでは省略。
        let mut s = State {
            ix: x,
            iy: y,
            it: theta,
            total_cost: MAX_COST,
            penalty: PROB_BASE,
            local_penalty: 0,
            final_state: false,
            optimal_action: None,
            free: false,
        };

        // free_ = (map.data[y*x_num + x] == 0)
        let idx0 = (y * x_num + x) as usize;
        s.free = map.data[idx0] == 0;
        if !s.free {
            return s;
        }

        // ★固有バグ: 境界判定が線形 pos のみ。ix2 が負/列外でも pos が [0,len) なら
        //   隣接行のセルを読む。本家 `map.data[iy*x_num + ix]` は `data[pos]` と同値。
        for ix2 in (-margin + x)..=(margin + x) {
            for iy2 in (-margin + y)..=(margin + y) {
                let pos: i64 = iy2 as i64 * x_num as i64 + ix2 as i64;
                if 0 <= pos && (pos as usize) < map.data.len() && map.data[pos as usize] != 0 {
                    s.penalty = (margin_penalty * PROB_BASE as f64) as u64 + PROB_BASE;
                }
            }
        }
        s
    }

    /// 本家 `State(int x, int y, int theta, unsigned int cost)`。
    pub fn from_cost(x: i32, y: i32, theta: i32, cost: u32) -> Self {
        let free = cost != 255;
        State {
            ix: x,
            iy: y,
            it: theta,
            total_cost: MAX_COST,
            penalty: if free { (cost as u64) << PROB_BASE_BIT } else { 0 },
            local_penalty: 0,
            final_state: false,
            optimal_action: None,
            free,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(width: i32, height: i32, data: Vec<i8>) -> OccupancyGrid {
        OccupancyGrid {
            width,
            height,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data,
        }
    }

    #[test]
    fn occupied_cell_is_not_free_and_returns_early() {
        // data[0] != 0 → not free, penalty 残置 (PROB_BASE)。
        let map = grid(2, 2, vec![100, 0, 0, 0]);
        let s = State::from_occupancy(0, 0, 0, &map, 0, 30.0, 2);
        assert!(!s.free);
        assert_eq!(s.penalty, PROB_BASE);
        assert_eq!(s.total_cost, MAX_COST);
    }

    #[test]
    fn free_cell_with_no_obstacle_in_margin_keeps_base_penalty() {
        let map = grid(3, 3, vec![0; 9]);
        let s = State::from_occupancy(1, 1, 0, &map, 1, 30.0, 3);
        assert!(s.free);
        assert_eq!(s.penalty, PROB_BASE);
    }

    #[test]
    fn free_cell_near_obstacle_gets_margin_penalty() {
        // 中央 free、隣接に障害物 → penalty = 30*PROB_BASE + PROB_BASE。
        let mut data = vec![0; 9];
        data[0] = 100; // (x=0,y=0) 障害物
        let map = grid(3, 3, data);
        let s = State::from_occupancy(1, 1, 0, &map, 1, 30.0, 3);
        assert!(s.free);
        assert_eq!(s.penalty, (30.0 * PROB_BASE as f64) as u64 + PROB_BASE);
    }

    #[test]
    fn margin_loop_row_crossing_bug_is_reproduced() {
        // ★バグ再現: x=0 の free セルで margin=1 とすると ix2=-1 が現れる。
        // iy2=1, ix2=-1 → pos = 1*width + (-1) = width-1 = 前の行(行0)の右端セル。
        // そこに障害物を置くと、列(x=-1)は本来マップ外なのに penalty が立つ。
        // width=3: pos=2 → data[2] (行0,x=2)。
        let mut data = vec![0; 9];
        data[2] = 100; // 行0,x=2 に障害物
        let map = grid(3, 3, data);
        // 対象セル (x=0, y=1)。margin=1 → ix2 ∈ {-1,0,1}, iy2 ∈ {0,1,2}。
        // iy2=0,ix2=2 は範囲外だが、iy2=1,ix2=-1 → pos=2 → data[2]!=0 でヒット。
        let s = State::from_occupancy(0, 1, 0, &map, 1, 30.0, 3);
        assert!(s.free);
        assert_eq!(
            s.penalty,
            (30.0 * PROB_BASE as f64) as u64 + PROB_BASE,
            "行跨ぎバグにより penalty が立つこと"
        );
    }

    #[test]
    fn from_cost_free_and_obstacle() {
        let free = State::from_cost(2, 3, 5, 100);
        assert!(free.free);
        assert_eq!(free.penalty, 100u64 << PROB_BASE_BIT);
        assert_eq!(free.total_cost, MAX_COST);

        let occ = State::from_cost(2, 3, 5, 255);
        assert!(!occ.free);
        assert_eq!(occ.penalty, 0);
    }
}
```

- [ ] **Step 2: `lib.rs` に追加**

```rust
pub mod state;

pub use state::State;
```

- [ ] **Step 3: テスト実行 (バグ再現確認込み)**

Run: `cd vi_rs && cargo test -p vi_reference state`
Expected: PASS (特に `margin_loop_row_crossing_bug_is_reproduced`)

- [ ] **Step 4: Commit**

```bash
git add vi_rs/vi_reference/src/state.rs vi_rs/vi_reference/src/lib.rs
git commit -m "feat(vi_reference): add State with faithful margin-penalty bug"
```

---

## Task 7: `value_iterator.rs` Part A — 構造体 + new + 索引 + cell_delta + no_noise

**Files:**
- Create: `vi_rs/vi_reference/src/value_iterator.rs`
- Modify: `vi_rs/vi_reference/src/lib.rs`

- [ ] **Step 1: 構造体・new・索引・幾何ヘルパ + テストを書く**

> 注: 先頭の `use` ブロックは**モジュール全体 (Task 8〜16)** で使う import をまとめて入れる。
> Task 7 時点では一部が「未使用 import」warning になるが、これは想定内 (このワークスペースは
> `-Dwarnings` 未設定なのでビルドは通る)。後続タスクで全て使われ、Task 17 で warning 無しを確認する。
> **実装者はこの段階で未使用 import を削除しないこと。**

`vi_rs/vi_reference/src/value_iterator.rs`:
```rust
//! 本家 `ValueIterator` 忠実移植 (フルパイプライン)。

use std::collections::BTreeMap;
use std::f64::consts::PI;

use crate::action::Action;
use crate::msg::{OccupancyGrid, Quaternion};
use crate::params::{MAX_COST, PROB_BASE, PROB_BASE_BIT, RESOLUTION_T_BIT, RESOLUTION_XY_BIT};
use crate::state::State;
use crate::state_transition::StateTransition;
use crate::sweep_status::SweepWorkerStatus;

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
        }
    }

    /// 本家 `toIndex(ix,iy,it) = it + ix*cell_num_t_ + iy*(cell_num_t_*cell_num_x_)`。
    pub fn to_index(&self, ix: i32, iy: i32, it: i32) -> i32 {
        to_index_raw(ix, iy, it, self.cell_num_x, self.cell_num_t)
    }

    /// 本家 `inMapArea`。
    pub fn in_map_area(&self, ix: i32, iy: i32) -> bool {
        ix >= 0 && ix < self.cell_num_x && iy >= 0 && iy < self.cell_num_y
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn to_index_layout() {
        // cell_num_x=4, cell_num_t=60。
        assert_eq!(to_index_raw(0, 0, 0, 4, 60), 0);
        assert_eq!(to_index_raw(0, 0, 5, 4, 60), 5);
        assert_eq!(to_index_raw(1, 0, 0, 4, 60), 60);
        assert_eq!(to_index_raw(0, 1, 0, 4, 60), 240);
    }

    #[test]
    fn cell_delta_negative_correction() {
        // xy_res=0.05。x=-0.01 → |x|/res=0.2 → floor 0 → x<0 → -0-1 = -1。
        let (ix, _, _) = cell_delta(-0.01, 0.0, 0.0, 0.05, 6.0);
        assert_eq!(ix, -1);
        // x=0.06 → 1.2 → floor 1。
        let (ix2, _, _) = cell_delta(0.06, 0.0, 0.0, 0.05, 6.0);
        assert_eq!(ix2, 1);
    }

    #[test]
    fn cell_delta_theta_absolute_not_normalized() {
        // t=366, t_res=6 → floor(61) = 61 (絶対、wrap しない)。
        let (_, _, it) = cell_delta(0.0, 0.0, 366.0, 0.05, 6.0);
        assert_eq!(it, 61);
    }

    #[test]
    fn no_noise_negative_theta_normalized_once() {
        // from_t=10, rot=-20 → to_t=-10 → +360 = 350。
        let (_, _, to_t) = no_noise_state_transition(0.0, -20.0, 0.0, 0.0, 10.0);
        assert!((to_t - 350.0).abs() < 1e-9);
    }

    #[test]
    fn no_noise_over_360_not_normalized() {
        // from_t=350, rot=20 → to_t=370 (>=360 は残す)。
        let (_, _, to_t) = no_noise_state_transition(0.0, 20.0, 0.0, 0.0, 350.0);
        assert!((to_t - 370.0).abs() < 1e-9);
    }

    #[test]
    fn no_noise_forward_uses_cos_sin() {
        // fw=0.3, from_t=0 → to_x=0.3, to_y=0。
        let (to_x, to_y, _) = no_noise_state_transition(0.3, 0.0, 0.0, 0.0, 0.0);
        assert!((to_x - 0.3).abs() < 1e-9);
        assert!(to_y.abs() < 1e-9);
    }
}
```

- [ ] **Step 2: `lib.rs` に追加**

```rust
pub mod value_iterator;

pub use value_iterator::ValueIterator;
```

- [ ] **Step 3: テスト実行**

Run: `cd vi_rs && cargo test -p vi_reference value_iterator`
Expected: PASS

- [ ] **Step 4: Commit**

```bash
git add vi_rs/vi_reference/src/value_iterator.rs vi_rs/vi_reference/src/lib.rs
git commit -m "feat(vi_reference): ValueIterator struct + index/geometry helpers"
```

---

## Task 8: 遷移生成 (サブセルサンプリング + θ並列)

**Files:**
- Modify: `vi_rs/vi_reference/src/value_iterator.rs`

- [ ] **Step 1: `compute_theta_transitions` + `set_state_transition` を追加 + テスト**

`value_iterator.rs` の free 関数群に追加 (`no_noise_state_transition` の後):
```rust
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
```

`impl ValueIterator` に追加:
```rust
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
```

テスト追加 (`mod tests` 内):
```rust
    #[test]
    fn prob_sum_equals_prob_base() {
        // 任意 action・θで prob 総和 = 64^3 = 262144 = PROB_BASE。
        let list = compute_theta_transitions(0.3, 0.0, 0, 0.05, 6.0);
        let total: i64 = list.iter().map(|s| s.prob as i64).sum();
        assert_eq!(total, super::PROB_BASE as i64);
    }

    #[test]
    fn forward_theta0_moves_in_x() {
        // 前進 fw=0.3, θ=0, res=0.05 → 主に dix≈6, diy=0, dit=0。
        let list = compute_theta_transitions(0.3, 0.0, 0, 0.05, 6.0);
        // 最頻バケット (prob 最大) を確認。
        let top = list.iter().max_by_key(|s| s.prob).unwrap();
        assert_eq!(top.diy, 0);
        assert_eq!(top.dit, 0, "θ=0 の前進は絶対 θ=0");
        assert!(top.dix >= 5 && top.dix <= 6, "dix was {}", top.dix);
    }

    #[test]
    fn rotation_dit_is_absolute_theta() {
        // 左回転 rot=+20, θ=0, t_res=6 → to_t≈20 → dit≈3 (絶対)、dix=diy=0。
        let list = compute_theta_transitions(0.0, 20.0, 0, 0.05, 6.0);
        let top = list.iter().max_by_key(|s| s.prob).unwrap();
        assert_eq!(top.dix, 0);
        assert_eq!(top.diy, 0);
        assert_eq!(top.dit, 3, "rot+20 → 絶対 θ index 3");
    }

    #[test]
    fn rotation_dit_absolute_at_theta30() {
        // θ=30 (index 5, t_res=6 → θ_origin=30°), 左回転 +20 → to_t≈50 → dit≈8 (絶対)。
        let list = compute_theta_transitions(0.0, 20.0, 5, 0.05, 6.0);
        let top = list.iter().max_by_key(|s| s.prob).unwrap();
        assert_eq!(top.dit, 8, "θ_origin30 + rot20 = 50° → index 8 (絶対)");
    }
```

- [ ] **Step 2: テスト実行**

Run: `cd vi_rs && cargo test -p vi_reference value_iterator`
Expected: PASS (`prob_sum_equals_prob_base`, `rotation_dit_is_absolute_theta`, `rotation_dit_absolute_at_theta30` がθ絶対の挙動を固定)

- [ ] **Step 3: Commit**

```bash
git add vi_rs/vi_reference/src/value_iterator.rs
git commit -m "feat(vi_reference): faithful sub-cell-sampling transition generation"
```

---

## Task 9: マップ取り込み (`set_map_with_occupancy_grid` / `set_map_with_cost_grid` / `set_state`)

**Files:**
- Modify: `vi_rs/vi_reference/src/value_iterator.rs`

- [ ] **Step 1: メソッド追加 + テスト**

`impl ValueIterator` に追加:
```rust
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

        self.set_state(map, safety_radius, safety_radius_penalty);
        self.set_state_transition();
        self.set_sweep_orders();
    }

    /// 本家 `setState`。
    fn set_state(&mut self, map: &OccupancyGrid, safety_radius: f64, safety_radius_penalty: f64) {
        self.states.clear();
        let margin = (safety_radius / self.xy_resolution).ceil() as i32;
        for y in 0..self.cell_num_y {
            for x in 0..self.cell_num_x {
                for t in 0..self.cell_num_t {
                    self.states.push(State::from_occupancy(
                        x,
                        y,
                        t,
                        map,
                        margin,
                        safety_radius_penalty,
                        self.cell_num_x,
                    ));
                }
            }
        }
    }

    /// 本家 `setMapWithCostGrid`。`margin` は本家にあるが未使用。
    pub fn set_map_with_cost_grid(
        &mut self,
        map: &OccupancyGrid,
        theta_cell_num: i32,
        safety_radius: f64,
        _safety_radius_penalty: f64,
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

        self.states.clear();
        let _margin = (safety_radius / self.xy_resolution).ceil() as i32; // 本家にあるが未使用
        for y in 0..self.cell_num_y {
            for x in 0..self.cell_num_x {
                // 本家 `(unsigned int)(map.data[x + cell_num_x_*y] & 0xFF)`。
                let cost = (map.data[(x + self.cell_num_x * y) as usize] as u8) as u32;
                for t in 0..self.cell_num_t {
                    self.states.push(State::from_cost(x, y, t, cost));
                }
            }
        }
        self.set_state_transition();
        self.set_sweep_orders();
    }
```

`mod tests` に追加 (テスト用の小マップビルダ + 確認)。`OccupancyGrid` は親モジュールの
`use` が `use super::*` 経由で入るため再 import しない:
```rust
    fn free_grid(w: i32, h: i32) -> OccupancyGrid {
        OccupancyGrid {
            width: w,
            height: h,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data: vec![0; (w * h) as usize],
        }
    }

    #[test]
    fn set_map_occupancy_populates_states_and_transitions() {
        let actions = vec![Action::new("forward", 0.3, 0.0, 0)];
        let mut vi = ValueIterator::new(actions, 1);
        let map = free_grid(3, 2);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);

        assert_eq!(vi.cell_num_x, 3);
        assert_eq!(vi.cell_num_y, 2);
        assert_eq!(vi.cell_num_t, 60);
        assert_eq!(vi.t_resolution, 6.0);
        assert_eq!(vi.states.len(), 3 * 2 * 60);
        // 各 action の θ ごとに遷移が生成されている。
        assert_eq!(vi.actions[0].state_transitions.len(), 60);
        let total: i64 = vi.actions[0].state_transitions[0]
            .iter()
            .map(|s| s.prob as i64)
            .sum();
        assert_eq!(total, super::PROB_BASE as i64);
    }

    #[test]
    fn set_map_cost_grid_free_and_obstacle() {
        let actions = vec![Action::new("forward", 0.3, 0.0, 0)];
        let mut vi = ValueIterator::new(actions, 1);
        let mut map = free_grid(2, 1);
        map.data = vec![0, 255i32 as i8]; // 1 つ目 free(cost0), 2 つ目 255
        vi.set_map_with_cost_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        // index (x=0): free。 (x=1): not free。
        let s0 = &vi.states[vi.to_index(0, 0, 0) as usize];
        let s1 = &vi.states[vi.to_index(1, 0, 0) as usize];
        assert!(s0.free);
        assert!(!s1.free);
    }
```

- [ ] **Step 2: テスト実行**

Run: `cd vi_rs && cargo test -p vi_reference value_iterator`
Expected: PASS

- [ ] **Step 3: Commit**

```bash
git add vi_rs/vi_reference/src/value_iterator.rs
git commit -m "feat(vi_reference): map ingestion (occupancy/cost grid) + setState"
```

---

## Task 10: 走査順生成 (`set_sweep_orders`)

**Files:**
- Modify: `vi_rs/vi_reference/src/value_iterator.rs`

- [ ] **Step 1: `set_sweep_orders` 追加 + テスト ([4]/[5] アンバランス検証)**

`impl ValueIterator` に追加:
```rust
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
    }
```

`mod tests` に追加:
```rust
    #[test]
    fn sweep_orders_structure() {
        let actions = vec![Action::new("forward", 0.3, 0.0, 0)];
        let mut vi = ValueIterator::new(actions, 1);
        let map = free_grid(2, 2);
        vi.set_map_with_occupancy_grid(&map, 4, 0.2, 30.0, 0.2, 10); // 小さい cell_num_t=4
        let total = (2 * 2 * 4) as usize;
        assert_eq!(vi.sweep_orders.len(), 6);
        assert_eq!(vi.sweep_orders[0].len(), total);
        assert_eq!(vi.sweep_orders[1].len(), total);
        // [2],[3] は逆順
        let rev0: Vec<i32> = vi.sweep_orders[0].iter().rev().cloned().collect();
        assert_eq!(vi.sweep_orders[2], rev0);
        // ★[4] = [0]全体 + [1]後半 (size = total + (total - half))
        let half = total / 2;
        assert_eq!(vi.sweep_orders[4].len(), total + (total - half));
        assert_eq!(&vi.sweep_orders[4][..total], &vi.sweep_orders[0][..]);
        assert_eq!(&vi.sweep_orders[4][total..], &vi.sweep_orders[1][half..]);
        // ★[5] = [1]前半
        assert_eq!(vi.sweep_orders[5], vi.sweep_orders[1][..half].to_vec());
    }

    #[test]
    fn sweep_orders_idempotent() {
        let actions = vec![Action::new("forward", 0.3, 0.0, 0)];
        let mut vi = ValueIterator::new(actions, 1);
        let map = free_grid(2, 2);
        vi.set_map_with_occupancy_grid(&map, 4, 0.2, 30.0, 0.2, 10);
        let len_before = vi.sweep_orders.len();
        vi.set_sweep_orders(); // 2 回目は no-op
        assert_eq!(vi.sweep_orders.len(), len_before);
    }
```

- [ ] **Step 2: テスト実行**

Run: `cd vi_rs && cargo test -p vi_reference value_iterator`
Expected: PASS

- [ ] **Step 3: Commit**

```bash
git add vi_rs/vi_reference/src/value_iterator.rs
git commit -m "feat(vi_reference): faithful sweep-order generation"
```

---

## Task 11: コスト計算 (`action_cost_raw` / `value_iteration_raw`、wrapping)

**Files:**
- Modify: `vi_rs/vi_reference/src/value_iterator.rs`

- [ ] **Step 1: コア free 関数 + メソッドラッパ + テスト (wrapping 含む)**

free 関数群に追加:
```rust
/// 本家 `actionCost`。★u64 オーバーフロー折り返しを `wrapping_*` で再現。
/// `dit` は絶対 θ なので `(dit + nt) % nt` で wrap (s.it は足さない)。
pub(crate) fn action_cost_raw(
    states: &[State],
    a: &Action,
    s: &State,
    cell_num_x: i32,
    cell_num_y: i32,
    cell_num_t: i32,
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
        let after = &states[to_index_raw(ix, iy, it, cell_num_x, cell_num_t) as usize];
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
pub(crate) fn value_iteration_raw(
    states: &mut [State],
    actions: &[Action],
    idx: usize,
    cell_num_x: i32,
    cell_num_y: i32,
    cell_num_t: i32,
) -> u64 {
    if !states[idx].free || states[idx].final_state {
        return 0;
    }
    let mut min_cost: u64 = MAX_COST;
    let mut min_action: Option<usize> = None;
    {
        let s = &states[idx];
        for (ai, a) in actions.iter().enumerate() {
            let c = action_cost_raw(states, a, s, cell_num_x, cell_num_y, cell_num_t);
            if c < min_cost {
                min_cost = c;
                min_action = Some(ai);
            }
        }
    }
    let old = states[idx].total_cost;
    let delta = (min_cost as i64) - (old as i64);
    states[idx].total_cost = min_cost;
    states[idx].optimal_action = min_action;
    delta.unsigned_abs()
}
```

`impl ValueIterator` に追加 (テスト/外部用ラッパ):
```rust
    /// 本家 `actionCost`。
    pub fn action_cost(&self, s: &State, a: &Action) -> u64 {
        action_cost_raw(
            &self.states,
            a,
            s,
            self.cell_num_x,
            self.cell_num_y,
            self.cell_num_t,
        )
    }

    /// 本家 `valueIteration` (states[idx] を更新)。
    pub fn value_iteration_at(&mut self, idx: usize) -> u64 {
        value_iteration_raw(
            &mut self.states,
            &self.actions,
            idx,
            self.cell_num_x,
            self.cell_num_y,
            self.cell_num_t,
        )
    }
```

`mod tests` に追加 (states/action を直接構築)。`State` は `use super::*` 経由で入る:
```rust
    fn mk_state(ix: i32, iy: i32, it: i32, free: bool, total: u64, penalty: u64) -> State {
        State {
            total_cost: total,
            penalty,
            local_penalty: 0,
            ix,
            iy,
            it,
            free,
            final_state: false,
            optimal_action: None,
        }
    }

    fn single_action(dix: i32, diy: i32, dit: i32, nt: usize) -> Action {
        let mut a = Action::new("a", 0.0, 0.0, 0);
        a.state_transitions = vec![Vec::new(); nt];
        for it in 0..nt {
            a.state_transitions[it].push(StateTransition::new(dix, diy, dit, super::PROB_BASE as i32));
        }
        a
    }

    #[test]
    fn action_cost_deterministic_neighbor() {
        // 2x1 マップ、θ=0。dix=+1 で隣 (free, total=5*PROB_BASE, penalty=PROB_BASE)。
        // cost = (5*PB + PB)*PB >>18 = 6*PB。
        let nt = 1usize;
        let nx = 2;
        let ny = 1;
        let states = vec![
            mk_state(0, 0, 0, true, super::MAX_COST, super::PROB_BASE),
            mk_state(1, 0, 0, true, 5 * super::PROB_BASE, super::PROB_BASE),
        ];
        let a = single_action(1, 0, 0, nt);
        let s = states[0].clone();
        let c = super::action_cost_raw(&states, &a, &s, nx, ny, nt as i32);
        assert_eq!(c, 6 * super::PROB_BASE);
    }

    #[test]
    fn action_cost_out_of_map_returns_max() {
        let nt = 1usize;
        let states = vec![mk_state(0, 0, 0, true, 0, 0)];
        let a = single_action(-1, 0, 0, nt); // dix=-1 → 範囲外
        let s = states[0].clone();
        let c = super::action_cost_raw(&states, &a, &s, 1, 1, nt as i32);
        assert_eq!(c, super::MAX_COST);
    }

    #[test]
    fn action_cost_obstacle_neighbor_returns_max() {
        let nt = 1usize;
        let states = vec![
            mk_state(0, 0, 0, true, 0, 0),
            mk_state(1, 0, 0, false, 0, 0), // not free
        ];
        let a = single_action(1, 0, 0, nt);
        let s = states[0].clone();
        let c = super::action_cost_raw(&states, &a, &s, 2, 1, nt as i32);
        assert_eq!(c, super::MAX_COST);
    }

    #[test]
    fn action_cost_overflow_wraps() {
        // 未到達 free 隣接 (total=MAX_COST) → MAX_COST*PROB_BASE が u64 を折り返す。
        // 期待値: (MAX_COST + PROB_BASE) を PROB_BASE 倍して wrap し >>18。
        let nt = 1usize;
        let penalty = super::PROB_BASE;
        let states = vec![
            mk_state(0, 0, 0, true, super::MAX_COST, penalty),
            mk_state(1, 0, 0, true, super::MAX_COST, penalty),
        ];
        let a = single_action(1, 0, 0, nt);
        let s = states[0].clone();
        let c = super::action_cost_raw(&states, &a, &s, 2, 1, nt as i32);
        // 手計算: term = (MAX_COST + PROB_BASE) wrapping_mul PROB_BASE; result = term >> 18。
        let term = (super::MAX_COST.wrapping_add(penalty)).wrapping_mul(super::PROB_BASE);
        let expected = term >> super::PROB_BASE_BIT;
        assert_eq!(c, expected);
        // 折り返しにより MAX_COST 未満になることを確認 (固有挙動)。
        assert!(c < super::MAX_COST, "overflow wrap should yield value < MAX_COST, got {c}");
    }

    #[test]
    fn value_iteration_picks_min_and_records_action() {
        // 3x1 マップ θ=0。中央 (idx=1) から action0:dix=+1(隣 total=9), action1:dix=-1(隣 total=4)。
        let nt = 1usize;
        let nx = 3;
        let ny = 1;
        let mut states = vec![
            mk_state(0, 0, 0, true, 4 * super::PROB_BASE, super::PROB_BASE),
            mk_state(1, 0, 0, true, super::MAX_COST, super::PROB_BASE),
            mk_state(2, 0, 0, true, 9 * super::PROB_BASE, super::PROB_BASE),
        ];
        let a0 = single_action(1, 0, 0, nt); // → 右 (total=9)
        let a1 = single_action(-1, 0, 0, nt); // → 左 (total=4)
        let actions = vec![a0, a1];
        let mid = 1usize;
        let d = super::value_iteration_raw(&mut states, &actions, mid, nx, ny, nt as i32);
        // 左 (4*PB + PB)*PB >>18 = 5*PB。右 = 10*PB。min = 5*PB、action1。
        assert_eq!(states[mid].total_cost, 5 * super::PROB_BASE);
        assert_eq!(states[mid].optimal_action, Some(1));
        // delta = |5*PB - MAX_COST|
        assert_eq!(d, super::MAX_COST - 5 * super::PROB_BASE);
    }

    #[test]
    fn value_iteration_skips_final_and_obstacle() {
        let nt = 1usize;
        let mut s_final = mk_state(0, 0, 0, true, super::MAX_COST, super::PROB_BASE);
        s_final.final_state = true;
        let mut states = vec![s_final];
        let actions: Vec<Action> = vec![single_action(1, 0, 0, nt)];
        let d = super::value_iteration_raw(&mut states, &actions, 0, 1, 1, nt as i32);
        assert_eq!(d, 0);
        assert_eq!(states[0].total_cost, super::MAX_COST); // 未更新
    }
```

- [ ] **Step 2: テスト実行**

Run: `cd vi_rs && cargo test -p vi_reference value_iterator`
Expected: PASS (特に `action_cost_overflow_wraps` が折り返し固有挙動を固定)

- [ ] **Step 3: Commit**

```bash
git add vi_rs/vi_reference/src/value_iterator.rs
git commit -m "feat(vi_reference): action_cost/value_iteration with faithful u64 wrap"
```

---

## Task 12: ゴール設定 (`set_goal` / `set_state_values`)

**Files:**
- Modify: `vi_rs/vi_reference/src/value_iterator.rs`

- [ ] **Step 1: メソッド追加 + テスト**

`impl ValueIterator` に追加:
```rust
    /// 本家 `setGoal`。goal_t を [0,360) に正規化し、final_state を再計算。
    pub fn set_goal(&mut self, goal_x: f64, goal_y: f64, goal_t: i32) {
        let mut gt = goal_t;
        while gt < 0 {
            gt += 360;
        }
        while gt >= 360 {
            gt -= 360;
        }
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

        for s in self.states.iter_mut() {
            s.total_cost = if s.final_state { 0 } else { MAX_COST };
            s.local_penalty = 0;
            s.optimal_action = None;
        }
    }
```

`mod tests` に追加:
```rust
    #[test]
    fn set_goal_normalizes_theta() {
        let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(3, 3);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        vi.set_goal(0.0, 0.0, -10);
        assert_eq!(vi.goal_t, 350);
        vi.set_goal(0.0, 0.0, 370);
        assert_eq!(vi.goal_t, 10);
        assert_eq!(vi.status, "calculating");
    }

    #[test]
    fn set_state_values_pins_goal_cell() {
        // goal をグリッド角 (0.5,0.5) に置く。final_state は「セルの両角がゴール半径内」
        // を要求するため、角を共有する 4 セルの遠い角 (距離 √2*0.05≈0.0707m) を包む
        // R=0.08 を使う。margin_theta=360 で全θ許容。
        let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(20, 20); // res=0.05 → 範囲 1.0m
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.08, 360);
        vi.set_goal(0.5, 0.5, 0); // セル角 (10,10)=(0.5,0.5)
        // (ix=10,iy=10): 左下角=ゴール(r0=0)、右上角 r1=0.005 < 0.08^2=0.0064 → final。
        let idx = vi.to_index(10, 10, 0) as usize;
        assert!(vi.states[idx].final_state);
        assert_eq!(vi.states[idx].total_cost, 0);
        // 遠方セル (0,0) は距離 ≫ R → final でない。
        let far = vi.to_index(0, 0, 0) as usize;
        assert!(!vi.states[far].final_state);
        assert_eq!(vi.states[far].total_cost, super::MAX_COST);
    }
```

- [ ] **Step 2: テスト実行**

Run: `cd vi_rs && cargo test -p vi_reference value_iterator`
Expected: PASS

- [ ] **Step 3: Commit**

```bash
git add vi_rs/vi_reference/src/value_iterator.rs
git commit -m "feat(vi_reference): set_goal/set_state_values with final_state checks"
```

---

## Task 13: 単スレッド worker + `finished` + `run_value_iteration` (収束)

**Files:**
- Modify: `vi_rs/vi_reference/src/value_iterator.rs`

- [ ] **Step 1: メソッド追加 + 収束テスト**

`impl ValueIterator` に追加:
```rust
    /// 本家 `valueIterationWorker`。単スレッド経路 (決定的・テスト基準)。
    /// `times` 回スイープ。`status` が canceled/goal なら中断。
    pub fn value_iteration_worker(&mut self, times: i32, id: i32) {
        self.thread_status.insert(id, SweepWorkerStatus::default());
        let order_idx = (id as usize) % self.sweep_orders.len();

        for j in 0..times {
            if let Some(st) = self.thread_status.get_mut(&id) {
                st.sweep_step = j + 1;
            }
            let mut max_delta: u64 = 0;
            let order_len = self.sweep_orders[order_idx].len();
            for k in 0..order_len {
                let i = self.sweep_orders[order_idx][k] as usize;
                let d = self.value_iteration_at(i);
                if d > max_delta {
                    max_delta = d;
                }
            }
            if let Some(st) = self.thread_status.get_mut(&id) {
                st.delta = (max_delta >> PROB_BASE_BIT) as f64; // ★二重シフト (報告用)
            }
            if self.status == "canceled" || self.status == "goal" {
                break;
            }
        }
        if let Some(st) = self.thread_status.get_mut(&id) {
            st.finished = true;
        }
    }

    /// 本家 `finished`。thread 0..thread_num の状態を集約。
    /// std::map operator[] の既定挿入を `entry().or_default()` で再現。
    pub fn finished(&mut self) -> (Vec<u32>, Vec<f64>, bool) {
        let n = self.thread_num as usize;
        let mut sweep_times = vec![0u32; n];
        let mut deltas = vec![0f64; n];
        let mut finish = true;
        for t in 0..self.thread_num {
            let st = self.thread_status.entry(t).or_default();
            sweep_times[t as usize] = st.sweep_step as u32;
            deltas[t as usize] = st.delta;
            finish &= st.finished;
        }
        (sweep_times, deltas, finish)
    }

    /// 価値反復を実行するエントリ。`thread_num<=1` は単スレッド (決定的)。
    /// `thread_num>1` は Task 14 のマルチスレッド経路を使う。
    pub fn run_value_iteration(&mut self, times: i32) {
        if self.thread_num <= 1 {
            self.value_iteration_worker(times, 0);
        } else {
            self.run_value_iteration_multithread(times);
        }
    }
```

`mod tests` に追加:
```rust
    #[test]
    fn single_thread_converges_on_small_free_map() {
        // 5x5 free マップ、goal を中央セルに。十分スイープして goal 隣接が確定する。
        let mut vi = ValueIterator::new(
            vec![
                Action::new("forward", 0.3, 0.0, 0),
                Action::new("back", -0.2, 0.0, 1),
                Action::new("right", 0.0, -20.0, 2),
                Action::new("rightfw", 0.2, -20.0, 3),
                Action::new("left", 0.0, 20.0, 4),
                Action::new("leftfw", 0.2, 20.0, 5),
            ],
            1,
        );
        let map = free_grid(5, 5);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        vi.set_goal(0.1, 0.1, 0); // セル (2,2) 付近

        vi.run_value_iteration(300);

        // 何らかの非 goal セルが MAX_COST 未満 (= 到達可能) になっていること。
        let reachable = vi.states.iter().any(|s| !s.final_state && s.total_cost < super::MAX_COST);
        assert!(reachable, "value should propagate from goal");

        // 2 回目の実行で値が変わらない (収束済み) ことを idempotent で確認。
        let before: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
        vi.run_value_iteration(50);
        let after: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
        assert_eq!(before, after, "converged values must be stable");
    }

    #[test]
    fn finished_aggregates_thread_status() {
        let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(3, 3);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        vi.set_goal(0.0, 0.0, 0);
        vi.value_iteration_worker(3, 0);
        let (sweeps, _deltas, finish) = vi.finished();
        assert_eq!(sweeps.len(), 1);
        assert_eq!(sweeps[0], 3);
        assert!(finish);
    }
```

- [ ] **Step 2: テスト実行 (Task 14 未実装のため一時的に `run_value_iteration_multithread` を stub)**

Task 14 で本実装する前にコンパイルを通すため、`impl ValueIterator` に一時 stub を追加:
```rust
    // Task 14 で本実装に差し替える。
    fn run_value_iteration_multithread(&mut self, times: i32) {
        self.value_iteration_worker(times, 0);
    }
```

Run: `cd vi_rs && cargo test -p vi_reference value_iterator`
Expected: PASS (`single_thread_converges_on_small_free_map`, `finished_aggregates_thread_status`)

- [ ] **Step 3: Commit**

```bash
git add vi_rs/vi_reference/src/value_iterator.rs
git commit -m "feat(vi_reference): single-thread worker + finished + run entry"
```

---

## Task 14: マルチスレッド worker (unsafe 共有 states、本家データ競合の再現)

**Files:**
- Modify: `vi_rs/vi_reference/src/value_iterator.rs`

- [ ] **Step 1: stub を本実装に差し替え + テスト**

`value_iterator.rs` の先頭 `use` の後に共有ポインタラッパを追加:
```rust
/// `*mut State` をスレッド間共有するためのラッパ。
/// SAFETY: 本家の non-atomic 共有 `states_` のデータ競合を**忠実再現**するための
/// 意図的な共有可変。`thread_num>1` は本家同様に非決定的 (技術的 UB、x86 で動く)。
#[derive(Clone, Copy)]
struct StatesPtr(*mut State);
unsafe impl Send for StatesPtr {}
unsafe impl Sync for StatesPtr {}
```

Task 13 で入れた stub を削除し、本実装に差し替え:
```rust
    /// 本家 `valueIterationWorker` をスレッドごとに spawn したマルチスレッド経路。
    /// 共有 `states` を生ポインタ経由で non-atomic 並行更新する (本家のデータ競合を再現)。
    /// `status`/`thread_status` は安全側で扱う (バッチ実行では status は不変)。
    fn run_value_iteration_multithread(&mut self, times: i32) {
        self.thread_status.clear();

        let n_states = self.states.len();
        let ptr = StatesPtr(self.states.as_mut_ptr());
        let cell_num_x = self.cell_num_x;
        let cell_num_y = self.cell_num_y;
        let cell_num_t = self.cell_num_t;
        let thread_num = self.thread_num;
        let actions = &self.actions;
        let sweep_orders = &self.sweep_orders;
        // バッチ実行中は status は不変なので break 条件を bool (Copy) で先に確定し、
        // 各スレッドクロージャへ move キャプチャする (String を多重 move できないため)。
        let stop = self.status == "canceled" || self.status == "goal";

        let results: Vec<(i32, SweepWorkerStatus)> = std::thread::scope(|scope| {
            let handles: Vec<_> = (0..thread_num)
                .map(|id| {
                    scope.spawn(move || {
                        // edition 2021 の disjoint closure capture 対策: `ptr.0` だけを使うと
                        // `*mut State` が直接キャプチャされ Send にならないため、ラッパ全体を再束縛して
                        // StatesPtr (Send) としてキャプチャさせる。
                        let ptr = ptr;
                        // SAFETY: 全スレッドが同一バッファを共有。本家のデータ競合を忠実再現。
                        let states: &mut [State] =
                            unsafe { std::slice::from_raw_parts_mut(ptr.0, n_states) };
                        let mut st = SweepWorkerStatus::default();
                        let order = &sweep_orders[(id as usize) % sweep_orders.len()];
                        for j in 0..times {
                            st.sweep_step = j + 1;
                            let mut max_delta: u64 = 0;
                            for &si in order.iter() {
                                let d = value_iteration_raw(
                                    states,
                                    actions,
                                    si as usize,
                                    cell_num_x,
                                    cell_num_y,
                                    cell_num_t,
                                );
                                if d > max_delta {
                                    max_delta = d;
                                }
                            }
                            st.delta = (max_delta >> PROB_BASE_BIT) as f64;
                            if stop {
                                break;
                            }
                        }
                        st.finished = true;
                        (id, st)
                    })
                })
                .collect();
            handles.into_iter().map(|h| h.join().unwrap()).collect()
        });

        for (id, st) in results {
            self.thread_status.insert(id, st);
        }
    }
```

`mod tests` に追加:
```rust
    #[test]
    fn multithread_converges_close_to_single_thread() {
        // 同一マップ・ゴールで、マルチスレッド (データ競合あり・非決定的) が
        // 単スレッドと同程度に値を伝播し、近い解へ収束することを確認 (bit 一致は要求しない)。
        let build = |threads: i32| {
            let mut vi = ValueIterator::new(
                vec![
                    Action::new("forward", 0.3, 0.0, 0),
                    Action::new("back", -0.2, 0.0, 1),
                    Action::new("left", 0.0, 20.0, 4),
                ],
                threads,
            );
            let map = free_grid(6, 6);
            vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
            vi.set_goal(0.1, 0.1, 0);
            vi
        };

        let mut single = build(1);
        single.run_value_iteration(500);

        let mut multi = build(4);
        multi.run_value_iteration(500);

        // thread_num>1 は本家同様データ競合で非決定的 → bit 一致は要求しない。
        // 「マルチスレッドも単スレッドと同程度に値を伝播し、折り返し garbage を残さない」ことを確認。
        let finite = |vi: &ValueIterator| {
            vi.states.iter().filter(|s| s.total_cost < super::MAX_COST).count()
        };
        let max_finite = |vi: &ValueIterator| {
            vi.states
                .iter()
                .map(|s| s.total_cost)
                .filter(|&c| c < super::MAX_COST)
                .max()
                .unwrap_or(0)
        };
        let sf = finite(&single);
        let mf = finite(&multi);
        assert!(sf > 0, "single-thread should propagate values");
        assert!(
            mf >= sf * 9 / 10,
            "multi-thread coverage should be close to single (single={sf}, multi={mf})"
        );
        assert!(
            max_finite(&multi) <= max_finite(&single) * 2,
            "multi-thread must not leave overflow-wrapped garbage values"
        );
    }

    #[test]
    fn multithread_finished_reports_all_threads() {
        let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 3);
        let map = free_grid(4, 4);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        vi.set_goal(0.0, 0.0, 0);
        vi.run_value_iteration(5);
        let (sweeps, _d, finish) = vi.finished();
        assert_eq!(sweeps.len(), 3);
        assert!(finish);
        assert!(sweeps.iter().all(|&s| s == 5));
    }
```

- [ ] **Step 2: テスト実行**

Run: `cd vi_rs && cargo test -p vi_reference value_iterator`
Expected: PASS

(任意・環境がある場合のみ) data race の確認:
Run: `cd vi_rs && RUSTFLAGS="-Zsanitizer=thread" cargo +nightly test -p vi_reference multithread 2>&1 | head` → ThreadSanitizer は data race を報告し得る (本家を忠実再現している証左)。失敗扱いにはしない。

- [ ] **Step 3: Commit**

```bash
git add vi_rs/vi_reference/src/value_iterator.rs
git commit -m "feat(vi_reference): multithread worker reproducing shared-states data race"
```

---

## Task 15: 出力 + `pos_to_action` + status メソッド

**Files:**
- Modify: `vi_rs/vi_reference/src/value_iterator.rs`
- Modify: `vi_rs/vi_reference/src/lib.rs`

- [ ] **Step 1: 出力型・メソッド追加 + テスト**

`value_iterator.rs` の末尾 (`impl` の外) に出力型を追加:
```rust
/// 本家 `valueFunctionWriter` / `policyWriter` 相当のプレーンデータ。
/// `layers[t]` は長さ `cell_num_x*cell_num_y`、索引 `iy*cell_num_x + ix`。
pub struct GridLayers {
    pub cell_num_x: i32,
    pub cell_num_y: i32,
    pub cell_num_t: i32,
    pub layers: Vec<Vec<f64>>,
}
```

`impl ValueIterator` に追加:
```rust
    /// 本家 `valueFunctionWriter`。各 θ 層に `total_cost/prob_base`。
    pub fn value_function_writer(&self) -> GridLayers {
        let (nx, ny, nt) = (self.cell_num_x, self.cell_num_y, self.cell_num_t);
        let mut layers = vec![vec![0f64; (nx * ny) as usize]; nt as usize];
        for t in 0..nt {
            let mut i = t;
            while (i as usize) < self.states.len() {
                let s = &self.states[i as usize];
                layers[t as usize][(s.iy * nx + s.ix) as usize] =
                    s.total_cost as f64 / PROB_BASE as f64;
                i += nt;
            }
        }
        GridLayers { cell_num_x: nx, cell_num_y: ny, cell_num_t: nt, layers }
    }

    /// 本家 `policyWriter`。各 θ 層に optimal_action の id (None は -1)。
    pub fn policy_writer(&self) -> GridLayers {
        let (nx, ny, nt) = (self.cell_num_x, self.cell_num_y, self.cell_num_t);
        let mut layers = vec![vec![0f64; (nx * ny) as usize]; nt as usize];
        for t in 0..nt {
            let mut i = t;
            while (i as usize) < self.states.len() {
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
                let index = self.to_index(x, y, it) as usize;
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

    /// 本家 `posToAction`。
    pub fn pos_to_action(&mut self, x: f64, y: f64, t_rad: f64) -> Option<usize> {
        let ix = ((x - self.map_origin_x) / self.xy_resolution).floor() as i32;
        let iy = ((y - self.map_origin_y) / self.xy_resolution).floor() as i32;
        let t = (180.0 * t_rad / PI) as i32;
        let it = (((t + 360 * 100) % 360) as f64 / self.t_resolution).floor() as i32;
        let index = self.to_index(ix, iy, it) as usize;
        if self.states[index].final_state {
            self.status = "goal".to_string();
            None
        } else if self.states[index].optimal_action.is_some() {
            self.states[index].optimal_action
        } else {
            None
        }
    }

    pub fn set_cancel(&mut self) {
        self.status = "canceled".to_string();
    }
    pub fn end_of_trial(&self) -> bool {
        self.status == "canceled" || self.status == "goal"
    }
    pub fn arrived(&self) -> bool {
        self.status == "goal"
    }
    pub fn set_calculated(&mut self) {
        if self.status != "canceled" {
            self.status = "calculated".to_string();
        }
    }
    pub fn is_calculated(&self) -> bool {
        self.status == "calculated"
    }
```

`lib.rs` の re-export を更新。**Task 7 で追加した `pub use value_iterator::ValueIterator;`
の行を次で置き換える** (二重 import を避ける):
```rust
pub use value_iterator::{GridLayers, ValueIterator};
```

`mod tests` に追加:
```rust
    #[test]
    fn make_value_function_map_wraps_to_i8() {
        let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(2, 2);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        // set_goal を呼ばない → 全セル free・total_cost=MAX_COST のまま。
        // cost=MAX_COST/PROB_BASE ≫ threshold(60) かつ free → 250 → i8 にラップして -6。
        let og = vi.make_value_function_map(60, 0.0, 0.0, 0.0);
        assert_eq!(og.width, 2);
        assert_eq!(og.height, 2);
        assert!(og.data.iter().all(|&v| v == (250u8 as i8)));
        assert_eq!(250u8 as i8, -6);
    }

    #[test]
    fn status_transitions() {
        let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        assert_eq!(vi.status, "init");
        vi.set_calculated();
        assert!(vi.is_calculated());
        vi.set_cancel();
        assert!(vi.end_of_trial());
        vi.set_calculated(); // canceled からは変えない
        assert_eq!(vi.status, "canceled");
    }

    #[test]
    fn policy_writer_marks_unset_as_minus_one() {
        let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(2, 2);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        vi.set_goal(0.0, 0.0, 0);
        let pol = vi.policy_writer();
        assert_eq!(pol.layers.len(), 60);
        // 未計算なので全 -1。
        assert!(pol.layers[0].iter().all(|&v| v == -1.0));
    }
```

- [ ] **Step 2: テスト実行**

Run: `cd vi_rs && cargo test -p vi_reference value_iterator`
Expected: PASS

- [ ] **Step 3: Commit**

```bash
git add vi_rs/vi_reference/src/value_iterator.rs vi_rs/vi_reference/src/lib.rs
git commit -m "feat(vi_reference): outputs, pos_to_action, status methods"
```

---

## Task 16: `ValueIteratorLocal` (`local.rs`)

**Files:**
- Create: `vi_rs/vi_reference/src/local.rs`
- Modify: `vi_rs/vi_reference/src/lib.rs`

- [ ] **Step 1: `local.rs` 作成 + テスト**

`vi_rs/vi_reference/src/local.rs`:
```rust
//! 本家 `ValueIteratorLocal` 忠実移植。`ValueIterator` を内包 (合成) し override を再定義。
//! local の `actionCostLocal` は本家 `actionCost` と完全同一なので base 経由で計算する。

use std::f64::consts::PI;

use crate::action::Action;
use crate::msg::{LaserScan, OccupancyGrid};
use crate::params::{PROB_BASE, PROB_BASE_BIT};
use crate::value_iterator::ValueIterator;

pub struct ValueIteratorLocal {
    pub base: ValueIterator,
    pub local_ix_min: i32,
    pub local_ix_max: i32,
    pub local_iy_min: i32,
    pub local_iy_max: i32,
    pub local_ixy_range: i32,
    pub local_xy_range: f64,
}

impl ValueIteratorLocal {
    /// 本家 `ValueIteratorLocal(actions, thread_num)`。
    pub fn new(actions: Vec<Action>, thread_num: i32) -> Self {
        Self {
            base: ValueIterator::new(actions, thread_num),
            local_ix_min: 0,
            local_ix_max: 0,
            local_iy_min: 0,
            local_iy_max: 0,
            local_ixy_range: 0,
            local_xy_range: 0.0,
        }
    }

    /// 本家 `ValueIteratorLocal::setMapWithOccupancyGrid`。base を呼んでから local window 初期化。
    pub fn set_map_with_occupancy_grid(
        &mut self,
        map: &OccupancyGrid,
        theta_cell_num: i32,
        safety_radius: f64,
        safety_radius_penalty: f64,
        goal_margin_radius: f64,
        goal_margin_theta: i32,
    ) {
        self.base.set_map_with_occupancy_grid(
            map,
            theta_cell_num,
            safety_radius,
            safety_radius_penalty,
            goal_margin_radius,
            goal_margin_theta,
        );
        self.local_xy_range = 1.0;
        self.local_ixy_range = (self.local_xy_range / self.base.xy_resolution) as i32;
        self.local_ix_min = 0;
        self.local_iy_min = 0;
        self.local_ix_max = self.local_ixy_range * 2;
        self.local_iy_max = self.local_ixy_range * 2;
    }

    /// 本家 `inLocalArea`。
    fn in_local_area(&self, ix: i32, iy: i32) -> bool {
        ix >= self.local_ix_min
            && ix <= self.local_ix_max
            && iy >= self.local_iy_min
            && iy <= self.local_iy_max
    }

    /// 本家 `valueIterationLocal` = `valueIteration` (actionCostLocal は actionCost と同一)。
    pub fn value_iteration_local(&mut self, idx: usize) -> u64 {
        self.base.value_iteration_at(idx)
    }

    /// 本家 `localValueIterationLoop`。local window 内を走査。
    pub fn local_value_iteration_loop(&mut self) {
        let nt = self.base.cell_num_t;
        for iix in self.local_ix_min..=self.local_ix_max {
            for iiy in self.local_iy_min..=self.local_iy_max {
                for iit in 0..nt {
                    let i = self.base.to_index(iix, iiy, iit) as usize;
                    self.value_iteration_local(i);
                }
            }
        }
    }

    /// 本家 `localValueIterationWorker`。status が canceled/goal の間 executing に書き換え、
    /// その後 status が canceled/goal になるまで local ループを回す (背景スレッド前提)。
    /// 注: 決定的テストでは `local_value_iteration_loop` を直接呼ぶこと。
    pub fn local_value_iteration_worker(&mut self, _id: i32) {
        while self.base.status == "canceled" || self.base.status == "goal" {
            self.base.status = "executing".to_string();
        }
        while self.base.status != "canceled" && self.base.status != "goal" {
            self.local_value_iteration_loop();
        }
    }

    /// 本家 `setLocalCost`。レーザヒット点周辺に local_penalty を設定/半減。
    pub fn set_local_cost(&mut self, msg: &LaserScan, x: f64, y: f64, t: f64) {
        let start_angle = msg.angle_min;
        let nt = self.base.cell_num_t;
        let (ox, oy, res) = (self.base.map_origin_x, self.base.map_origin_y, self.base.xy_resolution);

        for i in 0..msg.ranges.len() {
            let a = t + msg.angle_increment * i as f64 + start_angle;
            let r = msg.ranges[i];
            let lx = x + r * a.cos();
            let ly = y + r * a.sin();
            let ix = ((lx - ox) / res).floor() as i32;
            let iy = ((ly - oy) / res).floor() as i32;

            // d = 0.1..=0.9 (本家 f64 刻みを忠実再現)
            let mut d = 0.1;
            while d <= 0.9 {
                let half_lx = x + r * a.cos() * d;
                let half_ly = y + r * a.sin() * d;
                let half_ix = ((half_lx - ox) / res).floor() as i32;
                let half_iy = ((half_ly - oy) / res).floor() as i32;
                if self.in_local_area(half_ix, half_iy) {
                    for it in 0..nt {
                        let index = self.base.to_index(half_ix, half_iy, it) as usize;
                        self.base.states[index].local_penalty /= 2;
                    }
                }
                d += 0.1;
            }

            for iix in (ix - 2)..=(ix + 2) {
                for iiy in (iy - 2)..=(iy + 2) {
                    if !self.in_local_area(iix, iiy) {
                        continue;
                    }
                    for it in 0..nt {
                        let index = self.base.to_index(iix, iiy, it) as usize;
                        self.base.states[index].local_penalty = 2048u64 << PROB_BASE_BIT;
                    }
                }
            }
        }
    }

    /// 本家 `setLocalWindow`。ロボット位置中心に local window をクランプ。
    pub fn set_local_window(&mut self, x: f64, y: f64) {
        let ix = ((x - self.base.map_origin_x) / self.base.xy_resolution).floor() as i32;
        let iy = ((y - self.base.map_origin_y) / self.base.xy_resolution).floor() as i32;
        let rng = self.local_ixy_range;
        self.local_ix_min = if ix - rng >= 0 { ix - rng } else { 0 };
        self.local_iy_min = if iy - rng >= 0 { iy - rng } else { 0 };
        self.local_ix_max = if ix + rng < self.base.cell_num_x {
            ix + rng
        } else {
            self.base.cell_num_x - 1
        };
        self.local_iy_max = if iy + rng < self.base.cell_num_y {
            iy + rng
        } else {
            self.base.cell_num_y - 1
        };
    }

    /// 本家 `ValueIteratorLocal::posToAction` (override)。
    pub fn pos_to_action(&mut self, x: f64, y: f64, t_rad: f64) -> Option<usize> {
        let ix = ((x - self.base.map_origin_x) / self.base.xy_resolution).floor() as i32;
        let iy = ((y - self.base.map_origin_y) / self.base.xy_resolution).floor() as i32;
        let t = (180.0 * t_rad / PI) as i32;
        let it = (((t + 360 * 100) % 360) as f64 / self.base.t_resolution).floor() as i32;
        let index = self.base.to_index(ix, iy, it) as usize;
        if self.base.states[index].final_state {
            self.base.status = "goal".to_string();
            None
        } else if self.base.states[index].optimal_action.is_some() {
            self.base.states[index].optimal_action
        } else {
            None
        }
    }

    /// 本家 `makeLocalValueFunctionMap`。
    pub fn make_local_value_function_map(
        &self,
        threshold: i32,
        x: f64,
        y: f64,
        yaw_rad: f64,
    ) -> OccupancyGrid {
        let nx_local = self.local_ixy_range * 2 + 1;
        let ny_local = self.local_ixy_range * 2 + 1;
        let it = ((((yaw_rad / PI * 180.0) as i32 + 360 * 100) % 360) as f64
            / self.base.t_resolution)
            .floor() as i32;
        let mut data: Vec<i8> = Vec::new();
        for yy in self.local_iy_min..=self.local_iy_max {
            for xx in self.local_ix_min..=self.local_ix_max {
                let index = self.base.to_index(xx, yy, it) as usize;
                let cost = self.base.states[index].total_cost as f64 / PROB_BASE as f64;
                let val: i32 = if cost < threshold as f64 {
                    (cost / threshold as f64 * 250.0) as i32
                } else if self.base.states[index].free {
                    250
                } else {
                    255
                };
                data.push(val as u8 as i8);
            }
        }
        OccupancyGrid {
            width: nx_local,
            height: ny_local,
            resolution: self.base.xy_resolution,
            origin_x: x - self.local_xy_range,
            origin_y: y - self.local_xy_range,
            origin_quat: self.base.map_origin_quat.clone(),
            data,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn free_grid(w: i32, h: i32) -> OccupancyGrid {
        OccupancyGrid {
            width: w,
            height: h,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data: vec![0; (w * h) as usize],
        }
    }

    #[test]
    fn set_map_initializes_local_window() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(60, 60); // res=0.05 → local_ixy_range = 1.0/0.05 = 20
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        assert_eq!(vi.local_ixy_range, 20);
        assert_eq!(vi.local_ix_max, 40);
        assert_eq!(vi.local_iy_max, 40);
    }

    #[test]
    fn set_local_window_clamps() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(60, 60);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        // ロボットを原点に → ix=iy=0、range=20 → min は 0 にクランプ、max は 20。
        vi.set_local_window(0.0, 0.0);
        assert_eq!(vi.local_ix_min, 0);
        assert_eq!(vi.local_iy_min, 0);
        assert_eq!(vi.local_ix_max, 20);
        assert_eq!(vi.local_iy_max, 20);
    }

    #[test]
    fn set_local_cost_sets_penalty_band() {
        let mut vi = ValueIteratorLocal::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
        let map = free_grid(60, 60);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        // 1 ビーム、正面 (angle_min=0, increment=0), range=0.5m → ヒット点 (10,0) 付近。
        let scan = LaserScan {
            angle_min: 0.0,
            angle_increment: 0.0,
            ranges: vec![0.5],
        };
        vi.set_local_cost(&scan, 0.0, 0.0, 0.0);
        // ヒット点±2 セルのどこかに 2048<<bit が立っていること。
        let hit = vi.base.to_index(10, 0, 0) as usize;
        assert_eq!(vi.base.states[hit].local_penalty, 2048u64 << PROB_BASE_BIT);
    }

    #[test]
    fn local_loop_runs_value_iteration_in_window() {
        let mut vi = ValueIteratorLocal::new(
            vec![
                Action::new("forward", 0.3, 0.0, 0),
                Action::new("left", 0.0, 20.0, 4),
            ],
            1,
        );
        let map = free_grid(60, 60);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        vi.base.set_goal(0.5, 0.5, 0); // window 内にゴール
        vi.set_local_window(0.5, 0.5);
        // local ループを数回回すと window 内の到達可能セルが伝播する。
        for _ in 0..50 {
            vi.local_value_iteration_loop();
        }
        let reachable = (vi.local_ix_min..=vi.local_ix_max).any(|xx| {
            (vi.local_iy_min..=vi.local_iy_max).any(|yy| {
                let idx = vi.base.to_index(xx, yy, 0) as usize;
                let s = &vi.base.states[idx];
                !s.final_state && s.total_cost < crate::params::MAX_COST
            })
        });
        assert!(reachable, "local VI should propagate value within window");
    }
}
```

- [ ] **Step 2: `lib.rs` に追加**

```rust
pub mod local;

pub use local::ValueIteratorLocal;
```

- [ ] **Step 3: テスト実行**

Run: `cd vi_rs && cargo test -p vi_reference local`
Expected: PASS

- [ ] **Step 4: Commit**

```bash
git add vi_rs/vi_reference/src/local.rs vi_rs/vi_reference/src/lib.rs
git commit -m "feat(vi_reference): ValueIteratorLocal (laser local penalty + local VI)"
```

---

## Task 17: 結合テスト + 全体検証 + warning クリーン

**Files:**
- Create: `vi_rs/vi_reference/tests/end_to_end.rs`

- [ ] **Step 1: 小マップ end-to-end 結合テスト**

`vi_rs/vi_reference/tests/end_to_end.rs`:
```rust
//! 本家 6 アクションでの小マップ end-to-end。

use vi_reference::params::MAX_COST;
use vi_reference::{Action, OccupancyGrid, ValueIterator};

fn default_actions() -> Vec<Action> {
    vec![
        Action::new("forward", 0.3, 0.0, 0),
        Action::new("back", -0.2, 0.0, 1),
        Action::new("right", 0.0, -20.0, 2),
        Action::new("rightfw", 0.2, -20.0, 3),
        Action::new("left", 0.0, 20.0, 4),
        Action::new("leftfw", 0.2, 20.0, 5),
    ]
}

fn free_grid(w: i32, h: i32) -> OccupancyGrid {
    OccupancyGrid {
        width: w,
        height: h,
        resolution: 0.05,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: Default::default(),
        data: vec![0; (w * h) as usize],
    }
}

#[test]
fn small_map_value_iteration_end_to_end() {
    let mut vi = ValueIterator::new(default_actions(), 1);
    let map = free_grid(8, 8);
    vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
    vi.set_goal(0.2, 0.2, 0); // セル (4,4) 付近

    vi.run_value_iteration(1000);

    // ゴール近傍に到達可能なセルが存在し、価値関数が抽出できる。
    let vf = vi.value_function_writer();
    assert_eq!(vf.layers.len(), 60);
    let threshold = MAX_COST as f64 / vi_reference::params::PROB_BASE as f64;
    let any_reachable = vf
        .layers
        .iter()
        .any(|layer| layer.iter().any(|&v| v < threshold));
    assert!(any_reachable, "value function should contain reachable cells");

    // policy も抽出できる。
    let pol = vi.policy_writer();
    assert_eq!(pol.layers.len(), 60);
}

#[test]
fn obstacle_cell_stays_max_cost() {
    // 障害物セルは not free → value_iteration がスキップ → total_cost は MAX_COST のまま。
    let mut data = vec![0i8; 5 * 5];
    data[(2 + 5 * 2) as usize] = 100; // 中央 (2,2) に障害物
    let map = OccupancyGrid {
        width: 5,
        height: 5,
        resolution: 0.05,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: Default::default(),
        data,
    };
    let mut vi = ValueIterator::new(default_actions(), 1);
    vi.set_map_with_occupancy_grid(&map, 60, 0.0, 30.0, 0.2, 10); // safety_radius=0 → margin 0
    vi.set_goal(0.0, 0.0, 0);
    vi.run_value_iteration(500);

    let obs = vi.to_index(2, 2, 0) as usize;
    assert!(!vi.states[obs].free, "obstacle cell must be not free");
    assert_eq!(
        vi.states[obs].total_cost,
        MAX_COST,
        "obstacle cell is skipped and stays at MAX_COST"
    );
}
```

- [ ] **Step 2: 結合テスト実行**

Run: `cd vi_rs && cargo test -p vi_reference --test end_to_end`
Expected: PASS

- [ ] **Step 3: クレート全テスト + warning チェック**

Run: `cd vi_rs && cargo test -p vi_reference`
Expected: 全 PASS

Run: `cd vi_rs && cargo build -p vi_reference 2>&1 | grep -i warning`
Expected: 出力なし (warning 無し)。warning が出たら該当箇所を修正 (未使用 import の削除等)。

- [ ] **Step 4: ワークスペース全体が壊れていないことを確認**

Run: `cd vi_rs && cargo test --workspace`
Expected: 全 PASS (既存 vi_core / vi_algorithm / vi_fixtures / vi_bench は無傷)

- [ ] **Step 5: clippy (任意だが推奨)**

Run: `cd vi_rs && cargo clippy -p vi_reference -- -D warnings`
Expected: PASS。残る lint は意図的なもの (`inherent_to_string`、`unsafe`) のみ局所 `#[allow]` で抑制済みであること。

- [ ] **Step 6: Commit**

```bash
git add vi_rs/vi_reference/tests/end_to_end.rs
git commit -m "test(vi_reference): end-to-end integration tests"
```

---

## 完了基準

- `cargo test -p vi_reference` 全 PASS、warning 無し。
- `cargo test --workspace` 全 PASS (既存無傷)。
- 仕様 §7 の固有挙動 (θ絶対index / サブセル262144 / margin 行跨ぎバグ / `to_t`負正規化のみ / sweep_orders アンバランス / u64 折り返し / i8 ラップ / 二重シフト) が各テストで固定されている。
- `ValueIterator` フルパイプライン + `ValueIteratorLocal` が公開 API として揃っている。

## 将来タスク (本計画の対象外)

- 本家バイナリとの実行時 bit 突き合わせ (ROS1 コンテナ起動が必要 — 比較ベンチ spec 側で扱う)。
- `vi_bench` からの呼び出し統合。
