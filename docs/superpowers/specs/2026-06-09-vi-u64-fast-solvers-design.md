# vi_reference u64 高速ソルバ群 + ベンチマーク 設計

- 日付: 2026-06-09
- 対象: `vi_rs/vi_reference`（本家 ROS1 `value_iteration` の u64 忠実移植）、`vi_compare`
- 関連: [vi-rs-algorithm-port-design](2026-05-22-vi-rs-algorithm-port-design.md)（u16 高速ソルバ群）、[vi-reference-faithful-port-design](2026-06-08-vi-reference-faithful-port-design.md)（u64 忠実移植）、[vi-ros-compare-benchmark-design](2026-06-04-vi-ros-compare-benchmark-design.md)（比較ベンチ基盤）

## 1. 背景と目的

`vi_algorithm`（u16 / 16bit データ契約）には複数の高速 VI ソルバ（frontier / block / pyramid 系）がある。これらは Reference と bit-exact に収束するが、**16bit 量子化モデル**で動くため本家 ROS1（u64 PROB_BASE 固定小数点モデル）とは数値が一致しない（先の f3d ベンチ: RMSE 6.78、方策一致 60%。差は純粋に 16bit↔u64 の量子化）。

本設計は、これら高速アルゴリズムを **本家と同一の u64 コストモデル上で実装**し、**本家と bit-exact かつ高速**であることを実証する。

### 1.1 核心となる観察

価値反復は更新順序に依らず一意の固定点へ収束する（min-アクションの単調縮小写像）。`vi_reference` の per-cell 更新 `value_iteration_raw`（`value_iterator.rs`）は**本家の Bellman 更新そのもの**であり、これを全グリッド走査ではなく **frontier / block の活性集合に対して呼ぶ**だけで、到達可能セルの固定点は本家と bit-exact になる。コスト数式（`action_cost_raw`: `(total_cost + penalty + local_penalty) × prob` を wrapping 積算 → `>> PROB_BASE_BIT`）には一切触れない。

### 1.2 スコープ

対象ソルバ（厳密ソルバ6種、すべて本家と bit-exact）:

| ソルバ | u16 移植元 | u64 での実体 |
|---|---|---|
| Reference | `reference/` | **既存**（`value_iteration_worker` 全走査）= ベンチ `ref` 側。再利用のみ |
| Frontier3D | `frontier/f3d.rs` | 新規（3D フロンティア） |
| Frontier2D | `frontier/f2d.rs` | 新規（2D 空間フロンティア、全 θ 更新） |
| FrontierStack | `frontier/stack.rs` | 新規（θ 層ごとの 2D フロンティア + θ OR マージ） |
| BlockRefine | `block/refine.rs` | 新規（ブロック単位スケジューラ + ブロック内全更新） |
| PyramidSweep | `block/pyramid.rs` | 新規（多解像度ピラミッド） |

**非対象**（今回実装しない）: 近似ソルバ（Frontier3DTau / TopK / CoarseTheta — 量子化前提で bit-exact にならない）、StreamMimic（16bit HLS カーネル模倣で u64 化は無意味）。

## 2. アーキテクチャ

### 2.1 配置

`vi_reference` に新モジュール `solvers/` を追加（`vi_reference` の依存ゼロ方針を維持）。

```
vi_rs/vi_reference/src/
  solvers/
    mod.rs        # Solver enum + 共通 run エントリ + frontier 基盤（bitset/dilation/seed）
    frontier3d.rs # Frontier3D
    frontier2d.rs # Frontier2D
    stack.rs      # FrontierStack
    block.rs      # BlockRefine
    pyramid.rs    # PyramidSweep
```

各ソルバは「セット済みの `ValueIterator`（`set_map_*` + `set_goal` 済み）」を受け取り、`states[].total_cost` / `optimal_action` を**収束まで**更新する。戻り値は反復回数・更新セル数・収束フラグ（`SolveStats` 相当の軽量 struct）。

### 2.2 再利用する本家更新

`value_iteration_raw(states, actions, idx, nx, ny, nt) -> u64`（既存, `pub(crate)`）を**そのまま**呼ぶ。これは:
- `final_state` / 非 `free` セルは更新せず 0 を返す（活性集合に混ざっても安全に無視される）
- min over アクションで `total_cost` を更新、`optimal_action` を設定
- 戻り値は `|new - old|`（変化量の絶対値）

**変化（changed）判定は「厳密減少」で取る**。u16 版フロンティアは `new_val < old`（厳密減少時のみ伝播）で判定する。これと bit 一致させるため、戻り値の絶対 delta ではなく**更新前後の `total_cost` を比較**して `after < before` のときだけ新フロンティアに追加する（`let before = states[idx].total_cost; value_iteration_raw(...); let after = states[idx].total_cost; if after < before { … }`）。到達可能セルでは値は単調減少するため実質 `delta>0 ⟺ 減少` だが、到達不能セルが近傍 wrapping で**増加**する稀ケースを誤って伝播させないため、符号を見る本方式を採る。`optimal_action` は `value_iteration_raw` が毎回（不変時も）再設定するので、近傍収束後の最終評価で本家と同一の argmin（同コスト時は最初のアクション勝ち）になる。

`solvers/` は `value_iterator` と同一クレートなので `pub(crate)` の `value_iteration_raw` / `action_cost_raw` / `to_index_raw` をそのまま使える（公開 API 変更不要）。

### 2.3 u64 フロンティア基盤（`solvers/mod.rs`）

u16 側 `vi_algorithm/src/bitboard` を移植せず、最小の活性集合表現を新規実装する。

- **活性集合**: `Bitset3D`（`Vec<u64>` ワード列、索引は本家 `to_index(ix,iy,it) = it + ix*nt + iy*nt*nx` と整合させる）。`set` / `test` / `popcount` / `enumerate` / `dilate(dx,dy,dt)` / `and`(passable) を実装。θ は循環（wrap）。
  - 注: 本家 `to_index` は θ が最内なので、空間 dilation はワード境界をまたぐ。実装簡潔性を優先し、まずは素直なセル単位 dilation（ビット技巧なしの集合演算）で正しさを担保する。性能が不足する場合のみワード並列化を後続最適化とする（YAGNI）。
- **dilation 変位** `(mx, my, mt)`: `actions` の全 `state_transitions[t]` を走査し `mx = max|dix|`, `my = max|diy|`, `mt = max circular_dist(dit, t)` を算出。`dit` は**絶対 θ**なので循環距離をとる。これは「あるセルが変化したとき再評価が必要な前駆セル集合」の正しい上位集合（over-approximation）を与える。
- **passable マスク**: `states[idx].free`。
- **goal/final マスク**: `states[idx].final_state`（更新対象から除外。`value_iteration_raw` 側でも 0 を返すので二重に安全）。
- **初期フロンティア種**: `total_cost < MAX_COST` のセル（= `set_goal` 後の `final_state` セル）。

### 2.4 各ソルバの移植マッピング

u16 ソルバ（`bellman_backup` を呼ぶ）→ u64 ソルバ（`value_iteration_raw` を呼ぶ）の機械的対応:

| u16 (vi_algorithm) | u64 (vi_reference solvers) |
|---|---|
| `ctx.value[[iy,ix,it]]`（u16） | `states[to_index(ix,iy,it)].total_cost`（u64） |
| `bellman_backup(...) -> u16`（new_val 計算） | `value_iteration_raw(...) -> delta`（in-place 更新 + 変化量） |
| `new_val < old` で changed 判定 | `delta > 0` で changed 判定 |
| `ctx.penalty != PENALTY_OBSTACLE` | `states[idx].free` |
| `ctx.goal_mask[[..]]` / `value < MAX_VALUE` | `states[idx].final_state` / `total_cost < MAX_COST` |
| `pin_goals`（goal を 0 に） | `set_goal`/`set_state_values` が既に実施済み（再 pin 不要） |
| `transitions.max_displacement()` | §2.3 の `(mx,my,mt)` 算出 |

各ソルバの骨格（フロンティア拡張 → 候補列挙 → per-cell 更新 → 新フロンティア構築 → 収束で停止）は u16 版と同型を保つ。Frontier3D を基準実装とし、Frontier2D（空間フロンティア・全 θ 更新）、FrontierStack（θ 層別 + θ OR マージ）、BlockRefine（ブロックスケジューラ + ブロック内 `local_sweeps` 回更新）、PyramidSweep（粗→細解像度）を続けて移植する。

### 2.5 公開 API

```rust
// vi_reference::solvers
pub enum U64Solver { Reference, Frontier3D, Frontier2D, FrontierStack, BlockRefine, PyramidSweep }
pub struct U64SolveStats { pub iters: u32, pub updates: u64, pub converged: bool }
/// セット済み ValueIterator を収束まで解く。max_iters は上限（フロンティア系は通常それ以前に収束）。
pub fn solve(vi: &mut ValueIterator, solver: U64Solver, max_iters: u32) -> U64SolveStats;
```

`Reference` は既存 `value_iteration_worker` を delta>>18==0 まで回すラッパ（または strict 固定点ループ）で表現する。

## 3. 正しさの検証（TDD）

各高速ソルバについて parity テストを書く（`vi_algorithm` の既存 parity テストと同型）:

1. 小マップ（占有/sentinel/空きの 3〜8 マス）で `ValueIterator` をセット。
2. Reference（全走査）を収束まで回した `states` をオラクルとする。
3. 同一初期状態から高速ソルバを収束まで回す。
4. **到達可能セル（`total_cost < REACH_THRESH`）について `total_cost` と `optimal_action` が bit 一致**することを assert。
   - 到達不能セルは Reference 側で wrapping 振動するため比較から除外（既存 strict 比較と同じ `REACH_THRESH = 1_000_000 * PROB_BASE`）。
   - 方策の tie-break（同コスト時の最初のアクション勝ち）も両者一致するはず。万一ずれる場合は「最終更新時の近傍が収束済みか」の差なので、その候補セルを再評価する処理で吸収する（実装時に parity テストで検出・対処）。

`make rs-test` に統合（`cargo test -p vi_reference`）。

## 4. ベンチマーク

### 4.1 ハーネス一般化

`vi_ref_bench`（`vi_reference/src/bin/`）を一般化、もしくは新 bin `vi_u64_bench` を追加し、**ソルバ名を引数**に取る:

```
vi_u64_bench <solver> <occ_raw> <width> <height> <resolution> <ox> <oy> \
             <goal_x> <goal_y> <goal_yaw_deg> <theta_cell_num> <safety_radius> \
             <safety_radius_penalty> <goal_margin_radius> <goal_margin_theta> \
             <max_sweeps> <out_dir>
```

- `<solver>` ∈ {reference, frontier3d, frontier2d, frontier_stack, block_refine, pyramid_sweep}
- 入力 occupancy は既存 `ref_bench.py` と同一（`to_occupancy` 共通）。
- 出力: `value_<solver>.npy` / `policy_<solver>.npy`（f64, `total_cost/PROB_BASE` 整数除算 = 本家 `valueFunctionWriter` と同一）/ `timing_<solver>.json`。ref と同形式なので compare.py と互換。

`reference` は既存 `vi_ref_bench` と同一出力（`value_ref.npy`）になるため、ベンチの `ref` 側と一致を二重確認できる。

### 4.2 比較パイプライン

- `compare.py` の `SIDES` を u64 各ソルバ用に拡張（任意 side 名を受け付ける汎用化、もしくは6エントリ追加）。各ソルバ `unreach=1e6`（u64 モデル）、`report_<solver>.md`。
- ベースラインは既存 strict `value_ros1.npy`（本家の真の固定点）。各 u64 ソルバ vs 本家 → **期待値 RMSE 0 / 方策 100%（bit-exact）+ 速度比**。
- 実行ドライバ: `vi_compare/u64/run_u64_bench.sh`（`run_ref_bench.sh` と同型、ソルバ名ループ）。Makefile に `compare-u64` / `compare-u64-report` を追加。
- 統合レポート `report_u64.md`: 6ソルバ × (elapsed / 反復 / 対本家 RMSE / 方策一致 / 速度比) の一覧表。先の `report_3way.md` と整合する形式。

### 4.3 期待される結果

- 全6ソルバ: 本家と **RMSE 0 / 方策 100%**（到達可能セル）。
- 速度: frontier/block 系は Reference より大幅に高速（u16 f3d で本家比 11x の実績）。u64 でも同等の高速化が出るはず（活性セルのみ更新するため）。これにより「高速アルゴリズムは本家と完全一致しつつ桁違いに速い」を実証。

## 5. 段階デリバリ

1. **基盤 + Frontier3D**: `solvers/mod.rs`（Bitset3D / dilation / seed）+ `frontier3d.rs` + parity テスト。`vi_u64_bench` に frontier3d/reference を実装しベンチで本家と bit-exact を実証。
2. **Frontier2D / FrontierStack**: 各 + parity テスト + ベンチ。
3. **BlockRefine / PyramidSweep**: 各 + parity テスト + ベンチ。
4. **統合**: compare.py 一般化、`run_u64_bench.sh`、Makefile ターゲット、`report_u64.md` 生成、全6ソルバの計測・報告。

各段階で `cargo test -p vi_reference` green を維持。

## 6. リスクと留意点

- **θ dilation（絶対θ）**: `dit` が絶対 θ のため循環距離 `mt` で上位集合をとる。over-approximation は正しさを損なわない（候補が増えるだけ）。過剰なら逆 θ 隣接表で精緻化（後続最適化）。
- **方策の tie-break 一致**: §3 参照。parity テストで担保し、ずれたら候補再評価で対処。
- **到達不能セル**: フロンティアは未到達セルに触れず `MAX_COST` 据置。Reference 全走査は wrapping 振動。比較は到達可能セルのみ（`REACH_THRESH`）なので問題なし。
- **性能**: 初版はセル単位 dilation（ビット技巧最小）。house.pgm（384×384×60）で実用速度が出ない場合のみワード並列 dilation を追加（YAGNI）。
- **vi_reference のスコープ拡大**: 「忠実移植 Reference のみ」から「u64 モデル + ソルバ群」へ拡張。コスト数式は不変なので忠実性は保たれる。lib.rs に `pub mod solvers;` を追加。

## 7. 非目標

- u16 `vi_algorithm` 側の変更（触らない）。
- 近似ソルバ / StreamMimic の u64 化。
- 並列（rayon）u64 ソルバ（単スレッド公平比較が目的。必要なら後続）。
- ピーク性能最適化（正しさ + bit-exact 実証が一次目的）。
