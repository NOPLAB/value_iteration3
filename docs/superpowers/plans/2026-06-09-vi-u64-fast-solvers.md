# vi_reference u64 高速ソルバ群 + ベンチマーク Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** vi_algorithm の厳密高速ソルバ6種（Reference, Frontier3D, Frontier2D, FrontierStack, BlockRefine, PyramidSweep）を、本家 u64 コストモデル（vi_reference）上で実装し、本家と bit-exact かつ高速であることをベンチで実証する。

**Architecture:** vi_reference に `solvers/` モジュールを追加。各高速ソルバは既存の per-cell 更新 `value_iteration_raw`（本家の正確な Bellman 更新）を、全走査ではなくフロンティア/ブロックの活性集合に対して呼ぶ。VI は更新順序に依らず同じ固定点へ収束するので到達可能セルは本家と bit-exact。検証は parity テスト（vs Reference 全走査）。ベンチは `vi_u64_bench`（ソルバ名引数）+ compare.py 一般化。

**Tech Stack:** Rust (vi_reference, 依存ゼロ), Python (compare.py, numpy), Docker (vi_ros2_dev:humble), Make.

設計: `docs/superpowers/specs/2026-06-09-vi-u64-fast-solvers-design.md`

---

## ファイル構成

新規:
- `vi_rs/vi_reference/src/solvers/mod.rs` — `U64Solver` enum, `solve()` dispatcher, `U64SolveStats`, フロンティア基盤（`Bitset3D`, `displacement()`, seed helper）
- `vi_rs/vi_reference/src/solvers/frontier3d.rs` — Frontier3D
- `vi_rs/vi_reference/src/solvers/frontier2d.rs` — Frontier2D
- `vi_rs/vi_reference/src/solvers/stack.rs` — FrontierStack
- `vi_rs/vi_reference/src/solvers/block.rs` — BlockRefine
- `vi_rs/vi_reference/src/solvers/pyramid.rs` — PyramidSweep
- `vi_rs/vi_reference/src/bin/vi_u64_bench.rs` — ソルバ名引数のベンチハーネス
- `vi_compare/u64/u64_bench.py` — occ 準備 + ハーネス起動（`ref/ref_bench.py` と同型）
- `vi_compare/u64/run_u64_bench.sh` — Docker 内でビルド+全ソルバ実行

変更:
- `vi_rs/vi_reference/src/lib.rs` — `pub mod solvers;`
- `vi_rs/vi_reference/src/value_iterator.rs` — `set_map_with_occupancy_grid` 等は既存 pub のまま利用。`value_iteration_raw` 等 `pub(crate)` は同一クレートなので可視。必要なら strict 収束ヘルパを追加。
- `vi_compare/compare/compare.py` — `SIDES` を u64 ソルバ用に拡張
- `Makefile` — `compare-u64` / `compare-u64-report`
- `vi_compare/results/report_u64.md`（生成物）

---

## Phase 1: 基盤 + Frontier3D + Reference + ベンチ（手法の end-to-end 実証）

### Task 1: Bitset3D（フロンティア活性集合）

**Files:**
- Create: `vi_rs/vi_reference/src/solvers/mod.rs`
- Modify: `vi_rs/vi_reference/src/lib.rs`（末尾に `pub mod solvers;`）

索引は本家 `to_index(ix,iy,it) = it + ix*nt + iy*nt*nx`（θ 最内）と整合。総セル数 `nx*ny*nt` のビット列を `Vec<u64>` で持つ。

- [ ] **Step 1: 失敗するテストを書く**（`solvers/mod.rs` の `#[cfg(test)]`）

```rust
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
```

- [ ] **Step 2: テスト失敗を確認** — Run: `cd vi_rs && cargo test -p vi_reference bitset_tests`  Expected: コンパイルエラー（`Bitset3D` 未定義）

- [ ] **Step 3: Bitset3D を実装**（`solvers/mod.rs` 冒頭）

```rust
//! u64 コストモデル上で動く高速 VI ソルバ群。各ソルバは本家の per-cell 更新
//! `value_iteration_raw` を活性集合に対して呼ぶ。コスト数式は不変なので、到達可能
//! セルの収束値は Reference (全走査) = 本家と bit-exact。
use crate::value_iterator::ValueIterator;

/// 索引 `it + ix*nt + iy*nt*nx`（本家 to_index と整合）のビット集合。
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
                } else { None }
            })
        })
    }
    /// 空間 ±dx,±dy（境界クリップ）と θ ±dt（循環 wrap）で膨張した集合を返す。
    pub(crate) fn dilate(&self, dx: i32, dy: i32, dt: i32) -> Bitset3D {
        let mut out = Bitset3D::new(self.nx, self.ny, self.nt);
        for (ix, iy, it) in self.enumerate() {
            for ddx in -dx..=dx {
                let jx = ix + ddx;
                if jx < 0 || jx >= self.nx { continue; }
                for ddy in -dy..=dy {
                    let jy = iy + ddy;
                    if jy < 0 || jy >= self.ny { continue; }
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
```

- [ ] **Step 4: テスト合格を確認** — Run: `cd vi_rs && cargo test -p vi_reference bitset_tests`  Expected: PASS

- [ ] **Step 5: コミット** — `git add vi_rs/vi_reference/src/solvers/mod.rs vi_rs/vi_reference/src/lib.rs && git commit -m "feat(vi_reference): u64 frontier Bitset3D"`

### Task 2: 変位算出 + seed + Reference/Frontier3D 用ヘルパ

**Files:** Modify `vi_rs/vi_reference/src/solvers/mod.rs`

- [ ] **Step 1: 失敗するテストを書く**

```rust
#[cfg(test)]
mod helper_tests {
    use super::*;
    use crate::action::Action;
    use crate::value_iterator::ValueIterator;
    use crate::msg::OccupancyGrid;
    fn small_vi() -> ValueIterator {
        // 5x5 全 free, theta=4。forward 0.3m など本家6アクション。
        let actions = vec![
            Action::new("forward", 0.3, 0.0, 0), Action::new("back", -0.2, 0.0, 1),
            Action::new("right", 0.0, -20.0, 2), Action::new("rightfw", 0.2, -20.0, 3),
            Action::new("left", 0.0, 20.0, 4), Action::new("leftfw", 0.2, 20.0, 5),
        ];
        let mut vi = ValueIterator::new(actions, 1);
        let map = OccupancyGrid { width: 5, height: 5, resolution: 0.05,
            origin_x: 0.0, origin_y: 0.0, origin_quat: Default::default(), data: vec![0i8; 25] };
        vi.set_map_with_occupancy_grid(&map, 4, 0.2, 30.0, 0.3, 15);
        vi.set_goal(0.10, 0.10, 0);
        vi
    }
    #[test]
    fn displacement_is_bounded_and_positive() {
        let vi = small_vi();
        let (mx, my, mt) = displacement(&vi);
        assert!(mx >= 1 && my >= 1);
        assert!(mt >= 0 && mt < vi.cell_num_t);
    }
    #[test]
    fn seed_contains_goal_cells() {
        let vi = small_vi();
        let seed = seed_frontier(&vi);
        // final_state セル (total_cost==0) が種に含まれる
        let n_final = vi.states.iter().filter(|s| s.final_state).count();
        assert!(n_final > 0);
        assert_eq!(seed.popcount(), n_final as u64);
    }
}
```

- [ ] **Step 2: テスト失敗を確認** — Run: `cd vi_rs && cargo test -p vi_reference helper_tests`  Expected: FAIL（`displacement`/`seed_frontier` 未定義）

- [ ] **Step 3: ヘルパを実装**（`solvers/mod.rs`）

```rust
use crate::params::MAX_COST;

/// dilation 変位 (mx,my,mt) を actions の全遷移から算出。dit は絶対θなので
/// 各 (action, source theta t) について循環距離 min(|dit-t|, nt-|dit-t|) を取り mt とする。
pub(crate) fn displacement(vi: &ValueIterator) -> (i32, i32, i32) {
    let nt = vi.cell_num_t;
    let (mut mx, mut my, mut mt) = (0i32, 0i32, 0i32);
    for a in &vi.actions {
        for (t, trans) in a.state_transitions.iter().enumerate() {
            for st in trans {
                mx = mx.max(st.dix.abs());
                my = my.max(st.diy.abs());
                let raw = (st.dit - t as i32).rem_euclid(nt);
                let circ = raw.min(nt - raw);
                mt = mt.max(circ);
            }
        }
    }
    (mx.max(1), my.max(1), mt)
}

/// 初期フロンティア種: total_cost < MAX_COST のセル（set_goal 後の final_state セル）。
pub(crate) fn seed_frontier(vi: &ValueIterator) -> Bitset3D {
    let mut bb = Bitset3D::new(vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    for s in &vi.states {
        if s.total_cost < MAX_COST {
            bb.set(s.ix, s.iy, s.it);
        }
    }
    bb
}
```

- [ ] **Step 4: テスト合格を確認** — Run: `cd vi_rs && cargo test -p vi_reference helper_tests`  Expected: PASS

- [ ] **Step 5: コミット** — `git commit -am "feat(vi_reference): u64 frontier displacement + seed helpers"`

### Task 3: Frontier3D + parity テスト

**Files:** Create `vi_rs/vi_reference/src/solvers/frontier3d.rs`; Modify `solvers/mod.rs`（`mod frontier3d;`）

移植元: `vi_rs/vi_algorithm/src/frontier/f3d.rs` の `run_serial_inner`。マッピングは spec §2.4。変化判定は §2.2（before/after の厳密減少）。

- [ ] **Step 1: 失敗する parity テストを書く**（`frontier3d.rs` の `#[cfg(test)]`）

```rust
#[cfg(test)]
mod tests {
    use crate::action::Action;
    use crate::msg::OccupancyGrid;
    use crate::value_iterator::ValueIterator;
    use crate::params::PROB_BASE;
    use crate::solvers::frontier3d::frontier3d_solve;

    const REACH: u64 = 1_000_000u64 * PROB_BASE;

    fn actions() -> Vec<Action> {
        vec![
            Action::new("forward", 0.3, 0.0, 0), Action::new("back", -0.2, 0.0, 1),
            Action::new("right", 0.0, -20.0, 2), Action::new("rightfw", 0.2, -20.0, 3),
            Action::new("left", 0.0, 20.0, 4), Action::new("leftfw", 0.2, 20.0, 5),
        ]
    }
    fn make_vi(w: i32, h: i32, occ: Vec<i8>) -> ValueIterator {
        let mut vi = ValueIterator::new(actions(), 1);
        let map = OccupancyGrid { width: w, height: h, resolution: 0.05,
            origin_x: 0.0, origin_y: 0.0, origin_quat: Default::default(), data: occ };
        vi.set_map_with_occupancy_grid(&map, 8, 0.2, 30.0, 0.3, 15);
        vi.set_goal(0.10, 0.10, 0);
        vi
    }
    /// Reference 全走査を strict 固定点まで回す（到達可能セルが変化しなくなるまで）。
    fn run_reference_to_fixed_point(vi: &mut ValueIterator) {
        let mut prev: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
        for _ in 0..2000 {
            vi.value_iteration_worker(1, 0);
            let mut changed = false;
            for (i, s) in vi.states.iter().enumerate() {
                if s.total_cost < REACH && s.total_cost != prev[i] { changed = true; }
                prev[i] = s.total_cost;
            }
            if !changed { break; }
        }
    }

    #[test]
    fn parity_empty_8x8() {
        let mut a = make_vi(8, 8, vec![0i8; 64]);
        let mut b = make_vi(8, 8, vec![0i8; 64]);
        run_reference_to_fixed_point(&mut a);
        frontier3d_solve(&mut b, 2000);
        for i in 0..a.states.len() {
            if a.states[i].total_cost < REACH {
                assert_eq!(a.states[i].total_cost, b.states[i].total_cost, "total_cost mismatch @ {i}");
                assert_eq!(a.states[i].optimal_action, b.states[i].optimal_action, "policy mismatch @ {i}");
            }
        }
    }
}
```

- [ ] **Step 2: テスト失敗を確認** — Run: `cd vi_rs && cargo test -p vi_reference parity_empty_8x8`  Expected: FAIL（`frontier3d_solve` 未定義）

- [ ] **Step 3: Frontier3D を実装**（`solvers/frontier3d.rs`）

```rust
//! Frontier3D の u64 版。vi_algorithm/src/frontier/f3d.rs の run_serial_inner を
//! 本家 u64 モデル（value_iteration_raw）へ移植。
use crate::solvers::{displacement, seed_frontier, Bitset3D};
use crate::value_iterator::{value_iteration_raw, ValueIterator};

/// セット済み ValueIterator を Frontier3D で収束まで解く。(iters, updates, converged) を返す。
pub fn frontier3d_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let (mx, my, mt) = displacement(vi);
    let mut frontier = seed_frontier(vi);
    let mut updates: u64 = 0;
    let mut iters: u32 = 0;
    while frontier.popcount() > 0 && iters < max_iter {
        iters += 1;
        let candidates = frontier.dilate(mx, my, mt);
        let mut new_frontier = Bitset3D::new(nx, ny, nt);
        for (ix, iy, it) in candidates.enumerate() {
            let idx = vi.to_index(ix, iy, it) as usize;
            // free/final は value_iteration_raw が 0 を返し更新しない（安全に無視）。
            let before = vi.states[idx].total_cost;
            value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
            let after = vi.states[idx].total_cost;
            if after < before {
                updates += 1;
                new_frontier.set(ix, iy, it);
            }
        }
        frontier = new_frontier;
    }
    (iters, updates, frontier.popcount() == 0)
}
```

`value_iteration_raw` を呼ぶため、`value_iterator.rs` で `pub(crate) fn value_iteration_raw` が `solvers` から見えることを確認（同一クレートなので可視。不可視なら `pub(crate)` を維持）。`solvers/mod.rs` に `pub mod frontier3d;` を追加。

- [ ] **Step 4: テスト合格を確認** — Run: `cd vi_rs && cargo test -p vi_reference parity_empty_8x8`  Expected: PASS（bit 一致）。失敗時は §6 リスク（方策 tie-break / θ dilation）を調査。

- [ ] **Step 5: 障害物マップの parity テストを追加**（`obstacle` と `sentinel` パターン）して PASS を確認、コミット — `git add -A && git commit -m "feat(vi_reference): Frontier3D u64 (bit-exact vs Reference)"`

### Task 4: U64Solver enum + solve() dispatcher

**Files:** Modify `vi_rs/vi_reference/src/solvers/mod.rs`

- [ ] **Step 1: 失敗するテストを書く**

```rust
#[cfg(test)]
mod solve_tests {
    use super::*;
    use crate::action::Action;
    use crate::msg::OccupancyGrid;
    use crate::value_iterator::ValueIterator;
    fn vi() -> ValueIterator {
        let actions = vec![Action::new("forward", 0.3, 0.0, 0), Action::new("back", -0.2, 0.0, 1),
            Action::new("right", 0.0, -20.0, 2), Action::new("rightfw", 0.2, -20.0, 3),
            Action::new("left", 0.0, 20.0, 4), Action::new("leftfw", 0.2, 20.0, 5)];
        let mut v = ValueIterator::new(actions, 1);
        let map = OccupancyGrid { width: 8, height: 8, resolution: 0.05, origin_x: 0.0,
            origin_y: 0.0, origin_quat: Default::default(), data: vec![0i8; 64] };
        v.set_map_with_occupancy_grid(&map, 8, 0.2, 30.0, 0.3, 15);
        v.set_goal(0.10, 0.10, 0);
        v
    }
    #[test]
    fn solve_reference_and_frontier3d_agree() {
        let mut a = vi(); let mut b = vi();
        solve(&mut a, U64Solver::Reference, 2000);
        solve(&mut b, U64Solver::Frontier3D, 2000);
        let reach = 1_000_000u64 * crate::params::PROB_BASE;
        for i in 0..a.states.len() {
            if a.states[i].total_cost < reach {
                assert_eq!(a.states[i].total_cost, b.states[i].total_cost);
            }
        }
    }
    #[test]
    fn solver_from_str() {
        assert!(matches!(U64Solver::from_name("frontier3d"), Some(U64Solver::Frontier3D)));
        assert!(U64Solver::from_name("nope").is_none());
    }
}
```

- [ ] **Step 2: テスト失敗を確認** — Run: `cd vi_rs && cargo test -p vi_reference solve_tests`  Expected: FAIL

- [ ] **Step 3: dispatcher を実装**（`solvers/mod.rs`）

```rust
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum U64Solver { Reference, Frontier3D, Frontier2D, FrontierStack, BlockRefine, PyramidSweep }

impl U64Solver {
    pub fn from_name(s: &str) -> Option<Self> {
        Some(match s {
            "reference" => U64Solver::Reference,
            "frontier3d" => U64Solver::Frontier3D,
            "frontier2d" => U64Solver::Frontier2D,
            "frontier_stack" => U64Solver::FrontierStack,
            "block_refine" => U64Solver::BlockRefine,
            "pyramid_sweep" => U64Solver::PyramidSweep,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct U64SolveStats { pub iters: u32, pub updates: u64, pub converged: bool }

/// Reference は全走査を strict 固定点（到達可能セルが不変）まで回す。
fn reference_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let thr = 1_000_000u64 * crate::params::PROB_BASE; // 到達可能とみなす上限 (compare.py の 1e6 と整合)
    let mut prev: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
    let mut iters = 0u32;
    let converged = loop {
        vi.value_iteration_worker(1, 0);
        iters += 1;
        let mut changed = false;
        for (i, s) in vi.states.iter().enumerate() {
            if s.total_cost < thr && s.total_cost != prev[i] { changed = true; }
            prev[i] = s.total_cost;
        }
        if !changed { break true; }
        if iters >= max_iter { break false; }
    };
    (iters, 0, converged)
}

pub fn solve(vi: &mut ValueIterator, solver: U64Solver, max_iter: u32) -> U64SolveStats {
    let (iters, updates, converged) = match solver {
        U64Solver::Reference => reference_solve(vi, max_iter),
        U64Solver::Frontier3D => crate::solvers::frontier3d::frontier3d_solve(vi, max_iter),
        U64Solver::Frontier2D => crate::solvers::frontier2d::frontier2d_solve(vi, max_iter),
        U64Solver::FrontierStack => crate::solvers::stack::frontier_stack_solve(vi, max_iter),
        U64Solver::BlockRefine => crate::solvers::block::block_refine_solve(vi, max_iter),
        U64Solver::PyramidSweep => crate::solvers::pyramid::pyramid_sweep_solve(vi, max_iter),
    };
    U64SolveStats { iters, updates, converged }
}
```

注: Phase 1 時点では未実装ソルバ（frontier2d 等）の `mod` 宣言が無いとビルドできないため、Task 4 の段階では `solve()` の match から未実装枝を一旦 `todo!()` ではなく**コメントアウト**し、Frontier3D/Reference のみ有効化する。Phase 2/3 で各 `mod` 追加時に枝を有効化する。

- [ ] **Step 4: テスト合格を確認** — Run: `cd vi_rs && cargo test -p vi_reference solve_tests`  Expected: PASS

- [ ] **Step 5: コミット** — `git commit -am "feat(vi_reference): U64Solver dispatcher (Reference + Frontier3D)"`

### Task 5: vi_u64_bench ハーネス

**Files:** Create `vi_rs/vi_reference/src/bin/vi_u64_bench.rs`

移植元: `vi_rs/vi_reference/src/bin/vi_ref_bench.rs`（occ 読み込み・map セット・npy 書き出しを流用）。差分は「第1引数 solver 名」「solve() 呼び出し」「出力名 `value_<solver>.npy`」。

- [ ] **Step 1: ハーネスを実装**（テスト不要なバイナリ。end-to-end は Task 6 のベンチで検証）

`vi_ref_bench.rs` をベースに:
- 引数: `<solver> <occ_raw> <width> <height> <resolution> <ox> <oy> <gx> <gy> <gyaw_deg> <theta_cell_num> <safety_radius> <safety_radius_penalty> <goal_margin_radius> <goal_margin_theta> <max_sweeps> <out_dir>`（vi_ref_bench から先頭に `<solver>` 追加、末尾 `delta_threshold` 削除）
- `let solver = vi_reference::solvers::U64Solver::from_name(&solver_name).expect("unknown solver");`
- map セット（vi_ref_bench と同一: `set_map_with_occupancy_grid` + `set_goal(gx,gy,(gyaw_deg as i32))`）
- `let t0 = Instant::now(); let stats = vi_reference::solvers::solve(&mut vi, solver, max_sweeps as u32); let elapsed = ...;`
- 値・方策取り出しは vi_ref_bench と同一（`s.total_cost / PROB_BASE` 整数除算で f64、policy は `optimal_action` の id）
- 出力: `value_<solver>.npy` / `policy_<solver>.npy`（f64, C-order, vi_ref_bench の `write_npy_f64` を流用）/ `timing_<solver>.json`（`side` = solver 名, `iters`/`updates`/`converged`/`elapsed_sec`/`thread_num:1`）

- [ ] **Step 2: ビルド確認** — Run: `cd /tmp && CARGO_TARGET_DIR=/tmp/u64t cargo build --release --manifest-path /home/nop/dev/mywork/value_iteration_new/vi_rs/Cargo.toml -p vi_reference --bin vi_u64_bench`（ホストの .cargo 汚染回避のため /tmp + --manifest-path。失敗時は Docker 内ビルドで確認）  Expected: ビルド成功

- [ ] **Step 3: コミット** — `git add -A && git commit -m "feat(vi_reference): vi_u64_bench harness (solver-name arg)"`

### Task 6: compare.py 一般化 + ベンチ実行 + bit-exact 実証

**Files:** Modify `vi_compare/compare/compare.py`; Create `vi_compare/u64/u64_bench.py`, `vi_compare/u64/run_u64_bench.sh`; Modify `Makefile`

- [ ] **Step 1: compare.py の SIDES に u64 ソルバを追加**（`ref` を雛形に6エントリ。`unreach=1e6`, `label`, `report=report_<solver>.md`, `model_note='本家と同一 u64 モデル → bit-exact を期待'`）。side 名: `u64_reference, u64_frontier3d, u64_frontier2d, u64_frontier_stack, u64_block_refine, u64_pyramid_sweep`（vfile=`value_<solver>.npy` 等）。

- [ ] **Step 2: u64_bench.py を作成** — `ref/ref_bench.py` を流用。CLI に solver 名を渡す（先頭引数）。occ 生成（to_occupancy）は完全同一。

- [ ] **Step 3: run_u64_bench.sh を作成** — Docker(vi_ros2_dev:humble) 内で `/tmp` から `cargo build --release --manifest-path .../vi_rs/Cargo.toml -p vi_reference --bin vi_u64_bench`（`CARGO_TARGET_DIR=/workspace/vi_compare/.cache/u64_target`）→ 各 solver で u64_bench.py 実行。

- [ ] **Step 4: Makefile に `compare-u64`（Docker で run_u64_bench.sh）と `compare-u64-report`（各 side で compare.py 実行 → report_u64.md 集約）を追加。**

- [ ] **Step 5: ベンチ実行** — `make compare-u64 && make compare-u64-report`。Expected: 各 u64 ソルバ vs 本家 RMSE 0 / 方策 100%。Frontier3D が Reference より高速。

- [ ] **Step 6: コミット** — `git add -A && git commit -m "feat(vi_compare): u64 solver benchmark pipeline + Frontier3D bit-exact result"`

---

## Phase 2: Frontier2D + FrontierStack

### Task 7: Frontier2D + parity

**Files:** Create `vi_rs/vi_reference/src/solvers/frontier2d.rs`; Modify `solvers/mod.rs`

移植元 `vi_rs/vi_algorithm/src/frontier/f2d.rs`。空間 2D フロンティア、活性 (ix,iy) で全 θ を更新。spec §2.4 のマッピング。

- [ ] **Step 1: parity テスト**（Task 3 と同型。`frontier2d_solve` を Reference 固定点と bit 比較、empty/obstacle/sentinel）
- [ ] **Step 2: 失敗確認** — `cargo test -p vi_reference frontier2d`
- [ ] **Step 3: 実装** — 2D Bitset（`Bitset2D` を mod.rs に追加 or Bitset3D を nt=1 で流用）。コア:

```rust
pub fn frontier2d_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let (mx, my, _mt) = displacement(vi);
    // 種: any theta で total_cost<MAX_COST の (ix,iy)
    let mut frontier = seed_frontier_2d(vi);
    let (mut updates, mut iters) = (0u64, 0u32);
    while frontier.popcount() > 0 && iters < max_iter {
        iters += 1;
        let cand = frontier.dilate(mx, my); // 空間のみ
        let mut nf = Bitset2D::new(nx, ny);
        for (ix, iy) in cand.enumerate() {
            let mut changed = false;
            for it in 0..nt {
                let idx = vi.to_index(ix, iy, it) as usize;
                let before = vi.states[idx].total_cost;
                value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
                if vi.states[idx].total_cost < before { updates += 1; changed = true; }
            }
            if changed { nf.set(ix, iy); }
        }
        frontier = nf;
    }
    (iters, updates, frontier.popcount() == 0)
}
```

`Bitset2D`（nt 無し版, メソッドは Bitset3D と同名 `new/set/test/popcount/enumerate/dilate(dx,dy)`）を mod.rs に実装。`seed_frontier_2d`: 各 (ix,iy) でいずれかの θ が `total_cost<MAX_COST` なら set。
- [ ] **Step 4: 合格確認**、コミット `git commit -am "feat(vi_reference): Frontier2D u64 (bit-exact)"`
- [ ] **Step 5: solve() の Frontier2D 枝を有効化**、`solve_tests` に Frontier2D 比較を追加、コミット

### Task 8: FrontierStack + parity

**Files:** Create `vi_rs/vi_reference/src/solvers/stack.rs`; Modify `solvers/mod.rs`

移植元 `vi_rs/vi_algorithm/src/frontier/stack.rs`。θ 層ごとの 2D フロンティア + θ 方向 ±mt の OR マージ。

- [ ] **Step 1: parity テスト**（同型, empty/obstacle/sentinel）
- [ ] **Step 2: 失敗確認**
- [ ] **Step 3: 実装** — `Vec<Bitset2D>`（θ 層）。各反復: 層ごと 2D dilate → θ ±mt 層を OR → passable & 非 final マスク → 候補で value_iteration_raw → 減少で new_frontier[it] に set。stack.rs の構造を u64 へ。
- [ ] **Step 4: 合格確認**、コミット `git commit -am "feat(vi_reference): FrontierStack u64 (bit-exact)"`
- [ ] **Step 5: solve() の FrontierStack 枝有効化、テスト追加、コミット**

---

## Phase 3: BlockRefine + PyramidSweep

### Task 9: BlockRefine + parity

**Files:** Create `vi_rs/vi_reference/src/solvers/block.rs`; Modify `solvers/mod.rs`

移植元 `vi_rs/vi_algorithm/src/block/refine.rs`。ブロック（既定 8×8）単位の活性スケジューラ。活性ブロックは全セル×θ を `local_sweeps`（既定2）回更新。残差閾値 0 で bit-exact。

- [ ] **Step 1: parity テスト**（empty/obstacle/sentinel）
- [ ] **Step 2: 失敗確認**
- [ ] **Step 3: 実装** — ブロック活性マスク `Vec<bool>`（n_bx×n_by）。種: goal を含むブロック。各反復: 活性ブロック集合を ±(rx,ry) 膨張 → 各活性ブロックの全 (ix,iy,it) を local_sweeps 回 value_iteration_raw、ブロック内最大減少>0 なら次反復も活性。`block_refine_solve(vi, max_iter)`。パラメータは refine.rs の既定（bw=8, local_sweeps=2, threshold=0）。
- [ ] **Step 4: 合格確認**、コミット `git commit -am "feat(vi_reference): BlockRefine u64 (bit-exact)"`
- [ ] **Step 5: solve() 枝有効化、テスト追加、コミット**

### Task 10: PyramidSweep + parity（⚠ 固定点非一意リスク）

**Files:** Create `vi_rs/vi_reference/src/solvers/pyramid.rs`; Modify `solvers/mod.rs`

移植元 `vi_rs/vi_algorithm/src/block/pyramid.rs`。粗→細の 2×2 空間ピラミッド。**リスク**: 確率的 backup の floor で固定点が非一意になり得る（pyramid.rs のコメント参照）。prolongation が過小評価だと Reference と僅差になる恐れ。

- [ ] **Step 1: parity テスト**（empty で先に確認。bit 一致しない場合は Step 3 で対処）
- [ ] **Step 2: 失敗確認**
- [ ] **Step 3: 実装** — pyramid.rs の構造（coarsen / 各レベル sweep / prolongate / 子ブロック降下）を u64 で。最finest レベルでは prolongation せず、収束は最finest を strict まで回して保証。**parity が bit 一致しない場合**: 最finest レベルを Reference 同等の全走査 strict 固定点まで回す（粗レベルは活性集合の初期化にのみ使う）ことで bit-exact を担保する。
- [ ] **Step 4: 合格確認**（empty/obstacle/sentinel）、コミット `git commit -am "feat(vi_reference): PyramidSweep u64 (bit-exact)"`
- [ ] **Step 5: solve() 枝有効化、テスト追加、コミット**

---

## Phase 4: 統合・全6ソルバ計測・報告

### Task 11: 全ソルバをベンチに統合し report_u64.md 生成

**Files:** Modify `vi_compare/u64/run_u64_bench.sh`（全6ソルバループ）、compare.py（6 side 確認）、Create `vi_compare/compare/make_u64_report.py`（集約）

- [ ] **Step 1: run_u64_bench.sh のソルバリストを6種に**（reference, frontier3d, frontier2d, frontier_stack, block_refine, pyramid_sweep）
- [ ] **Step 2: make_u64_report.py を作成** — 各 `timing_<solver>.json` + compare.py 出力を読み、表「ソルバ / elapsed / iters / 対本家 RMSE / 方策一致 / 速度比 / bit-exact」を `report_u64.md` に集約。
- [ ] **Step 3: フル実行** — `make compare-u64 && make compare-u64-report`。Expected: 全6ソルバ RMSE 0 / 方策 100%、frontier/block/pyramid は reference より高速。
- [ ] **Step 4: report_u64.md を確認し、u64_reference == ref（既存 value_ref.npy）の二重一致を検証。** コミット `git add -A && git commit -m "feat(vi_compare): full u64 solver suite benchmark + report_u64.md"`

### Task 12: vi_node バグ修正の検証（Phase 1 で同時実施可）

**Files:** （既に修正済み `vi_ros2/vi_node/src/main.rs`）

- [ ] **Step 1: vi_node を frontier3d で走らせ収束確認** — `make compare-ros2`（vi_node_params.yaml の solver を一時的に frontier3d に）または既存 ros2 経路で、frontier 系が**最後まで収束**（sweeps が partial でない）することを確認。Expected: 修正前の sweeps=5/converged=False が解消。
- [ ] **Step 2: 既存 ros2(reference) の結果が不変であることを確認**（value_ros2.npy が修正前後で一致 = Reference 挙動は不変）。
- [ ] **Step 3: コミット** — `git commit -am "fix(vi_node): rely on worker converged flag, not final_delta==0 (frontier solvers)"`

---

## リスクと留意点（spec §6 より）

- **方策 tie-break**: parity テストで `optimal_action` まで bit 比較。ずれたら候補の最終再評価で吸収。
- **θ dilation（絶対θ）**: 循環距離 mt で上位集合。over-approx は正しさを損なわない。
- **PyramidSweep 固定点非一意**: 最finest を strict 全走査で締める（Task 10 Step 3）。
- **到達不能セル**: 比較は `value<1e6` のみ。フロンティアは未到達セルに触れない。
- **性能**: 初版はセル単位 dilation。house.pgm で実用速度が出なければワード並列化（YAGNI）。
