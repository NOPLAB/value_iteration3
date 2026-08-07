# vi_reference: 全ソルバ共通の solve 観測・制御抽象 (SolveObserver) 設計

日付: 2026-08-08
対象: `vi_rs/vi_reference` (solvers 全体)、`vi_ros2/vi_planner`・`vi_global_planner`・`vi_node` (追従移行)

## 背景と問題

vi_planner は名目上 `U64Solver::from_name` の全ソルバを受けるが、実際に成立するのは
一部だけである。原因は `PlannerCore::solve_dense` のチャンクループが solver に課す
**暗黙の契約**にある:

1. **チャンク再入で進捗が保存されること。** `solve(vi, solver, chunk)` を繰り返す方式
   なので、solver が非収束リターン時に `vi.states` へ書き戻さないと進捗が消える。
   fused/sparse は無条件 write_back で成立しているが、そのために毎チャンク
   `Geom::build` + `Fused::build_direct` + write_back + 全セル argmin
   (`final_policy_fused` は非収束時 `skip_unreached` 不可) という O(全状態数) の固定費を
   払い直している。
2. **境界で値と方策が観測可能なこと。** cancel 観測・`on_chunk` の途中経過 publish・
   `early_start` の `reaches_goal` がすべて依存する。core/mod.rs のコメントが
   「`frontier2d_sparse` が毎回 argmin パスを回すから読める」と solver 名を名指し
   している = 契約が型で表現されていない。
3. **priority 系はチャンク化不能。** `priority_solve` の `max_iter` は
   `pop_cap = n·max_iter` の暴走ガードでしかなく、1 呼び出しでヒープが尽きるまで
   走り切る。選ぶと solve 中のプリエンプトが効かず、再入するとヒープを失う。
4. **compact は別入口。** `solve_compact_mapped_stopping` は cancel (`AtomicBool`) と
   打ち切り (`stop: FnMut(&dyn CompactSink) -> bool`) を既にコールバックで持ち、
   planner は同じ制御を dense (手書きループ) と compact (クロージャ) の 2 通りに書く。

さらに sparse の `Snapshotter` (リーダーが N ラウンドごとにダンプ) を含め、
「solve の途中に外から介入する」仕組みが ad-hoc に 3 つ生えている。

## 設計: 制御反転 (Observer) + 遅延プローブ + 能力宣言

**セッション型 (begin/step/finish) にせず、solve の中へフックを渡す制御反転にする。**

- 並列エンジン (`async_gs_engine`, sparse 独自エンジン) は `std::thread::scope` で
  ワーカーを抱えるため、呼び出し側へ制御を返すセッション型はスコープを跨げない。
  制御反転なら「リーダーが境界でフックを呼ぶ」だけで、Snapshotter の一般化になる。
- 最も複雑な compact が既にこの形 (cancel + stop コールバック)。
- チャンク再入という機構自体が消えるので「進捗保存」要求が消滅し、fused/sparse の
  毎チャンク再ビルドと priority のヒープ喪失も同時に解決する。

### トレイト (`vi_reference::solvers::observe`)

```rust
pub enum SolveFlow {
    Continue,
    /// ここで打ち切る (early_start)。場は観測可能な整合状態のまま戻る (outcome.stopped)。
    Stop,
    /// プリエンプト。最短経路で戻る。場の内容は保証しない (outcome.cancelled)。
    Cancel,
}

/// 境界での場の観測窓。policy() を呼んだときだけ内部表現からの具現化
/// (write_back + argmin) が走る — 呼ばなければ無料。
pub trait SolveProbe {
    fn iters(&self) -> u32;      // 本家 iteration 換算の進捗
    fn updates(&self) -> u64;
    fn policy(&mut self) -> &dyn PolicyView;
}

/// solve の進行を外から制御するフック。solver は自然な境界
/// (ラウンド / スイープ / ブロックパス / バンド finalize / interval 相当の pop) ごとに呼ぶ。
pub trait SolveObserver {
    fn interval(&self) -> u32 { 1 }   // 観測間隔ヒント (本家 iteration 換算)
    fn boundary(&mut self, probe: &mut dyn SolveProbe) -> SolveFlow;
}

pub struct SolveOutcome {
    pub iters: u32,
    pub updates: u64,
    pub converged: bool,
    pub stopped: bool,    // Stop で打ち切った (場は使える)
    pub cancelled: bool,  // Cancel で中断した (場は破棄)
}

pub fn solve_observed(vi, solver, max_iter, obs: &mut dyn SolveObserver) -> SolveOutcome;
// 既存 solve() は NullObserver を渡す薄いラッパとして完全互換で残す。
```

observer は**常に呼び出しスレッドで呼ばれる** (並列エンジンはリーダー w==0 を spawn
せず呼び出しスレッドでインライン実行する)。`Send` 境界が不要になり、rclrs の
publish クロージャをそのまま持ち込める。

### 能力宣言 `U64Solver::caps()`

```rust
pub enum Partiality {
    /// 途中の場は全セル V ≥ v* の単調上界 (MAX_COST から降下)。貪欲降下は循環しない。
    UpperBound,
    /// 確定済みセルは v* に bit-exact、未確定は MAX_COST (compact)。
    ExactPrefix,
}
pub struct SolverCaps {
    pub exact: bool,       // 収束値が本家と bit-exact (prio_ls / 実パラメタ付き近似系は false)
    pub partial: Partiality,
    pub out_of_core: bool, // states を確保しない (PlanConfig::use_compact の実体)
    pub parallel: bool,    // VI_THREADS を消費する
}
```

`exact` はパラメタ依存: `Tau{tau:0}` / `TopK{k:u32::MAX}` / `CoarseTheta{step:1}` は
exact、実パラメタ付きは approximate。`PrioLS` は settle-once 近似なので常に false。
Partiality は early_start に対して両変種とも健全なので、ゲートではなく診断・
ドキュメントの型化 (approximate solver 選択時の起動警告など)。

### PolicyView 拡張

`fn value_at(&self, ix, iy, it) -> u64` を追加 (ValueIterator = states、CompactPolicy
= sink、境界プローブは write_back 済み vi)。`value_grid_of` を
`value_grid_on(&dyn PolicyView, ..)` に一般化し、途中経過 publish もロールアウト判定も
同じビュー 1 枚で済ませる (vi_planner の compact 分岐が既にこの形)。

### フック挿入箇所 — 10 箇所で 19 変種

| 挿入箇所 | 覆う solver | 境界 | プローブ |
|---|---|---|---|
| `solvers/original` (旧 reference_solve) | reference, stream_mimic | スイープ | vi そのまま (in-place) |
| `frontier3d_driver` | frontier3d, tau, topk, coarse_theta | ラウンド | vi そのまま |
| `frontier2d_driver` | frontier2d, soa, pad | ラウンド | soa/pad は要求時 write_back |
| `block_refine_sized` | block_refine, pyramid_sweep | 外側反復 | vi そのまま |
| `stack` | frontier_stack | ラウンド | vi そのまま |
| `frontier2d_par` | par | ラウンド | 要求時 write_back |
| `async_gs_engine` | par_unsafe, fused | ラウンド (リーダーインライン) | 要求時 write_back |
| sparse エンジン | sparse | ラウンド (Snapshotter の一般化) | 要求時 write_back |
| `priority_solve` | prio_ls, prio_lc | interval 相当の pop ごと | vi そのまま (ヒープ保持) |
| compact core | sparse_compact | バンド finalize (既存 stop/cancel をアダプト) | `CompactPolicy` (sink) |

`frontier2d_driver` は cell クロージャと boundary が同じモデルを可変借用する二重借用に
なるため、「cell + boundary の 2 メソッドを持つ 1 オブジェクト」を受ける形に変える。
要求時 write_back プローブは O(n)/観測だが観測は planner の `solve_chunk` 周期のみで、
今のチャンク方式が毎チャンク無条件に払っていた費用が「呼ばれたときだけ」になる
(cp/pen 上の遅延 argmin ビューはさらなる最適化として将来課題)。

### 本家 original の置き場

本家全走査ソルバ (旧 `reference_solve`) は `solvers/original/` へ移す。
`value_iterator.rs` (ValueIterator = 共有データモデル) は solver ではないので動かさない。

## 共通 conformance テスト (`solvers/conformance.rs`)

全変種 (近似は実パラメタでも: `Tau{tau>0}` / `TopK{k 小}` / `CoarseTheta{step:2}` /
`PrioLS`) をパラメタライズし、caps 駆動で期待を切り替える:

1. **収束**: 標準 3 マップ (empty / obstacle / sentinel) で converged。
2. **正しさ**: `caps().exact` → Reference 固定点と bit-exact (値+方策)。
   近似 → 到達可能セルで V ≥ v* (上界) かつ複数始点からのロールアウトがゴール到達。
3. **cancel 即時性**: 最初の境界で Cancel → cancelled=true、iters が interval 近傍。
4. **境界不変条件**: 各境界で probe の値が V ≥ v* (UpperBound) / 非 MAX セルは v* に
   一致 (ExactPrefix)。
5. **Stop 健全性**: `reaches_goal(from)` で Stop した場の上で `rollout_path_on(from)`
   が成功 (early_start の核心不変条件を全 solver に拡張)。
6. **ラッパ同値**: NullObserver の solve_observed ≡ solve() (bit 一致)。

散在する `parity_standard_maps_*` テストはこのスイートに置き換える (solver 固有の
エッジテストは残す)。並列 solver は 32×24 マップでも回す。

## プランナ移行 (Phase 3)

`solve_dense` のチャンクループと compact の stop クロージャを observer 実装 1 つ
(`SolveDirector { cancel, interval=solve_chunk, from, on_progress }`) に畳む。
`on_chunk` は `FnMut(&dyn PolicyView)` になり、compact でも solve 中の
`value_function` publish が動くようになる。`use_compact()` は `caps().out_of_core`。
`vi_global_planner::core` と `vi_node::sweep_thread` も同型に移行。

## 段階

1. observe 基盤 + caps + original 移動 + 直列系フック + conformance (rs-test 緑)
2. 並列エンジンのリーダーインライン化 + priority + compact アダプタ (rs-test 緑)
3. プランナ 3 ノード移行 (ホストは分離クレート、最終確認は Docker colcon)
