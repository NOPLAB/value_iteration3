# vi_rs: MATLAB アルゴリズムの Rust 移植 設計仕様

- 日付: 2026-05-22
- 対象ディレクトリ: `vi_rs/`
- 関連仕様:
  - `docs/superpowers/specs/2026-04-13-matlab-hdl-coder-streaming-design.md`
  - `docs/superpowers/specs/2026-04-14-matlab-src-refactor-design.md`
- ステータス: ドラフト（実装計画は別ファイルで後続）

## 1. 目的とスコープ

### 1.1 目的

`vi_matlab/` に実装されている価値反復アルゴリズム群を Rust に移植し、
ベンチマーク基盤を整える。狙いは以下の3点。

1. **CPU ベースライン**: MATLAB の三重ループ実装は遅く、FPGA との速度比較
   や `ms あたり何セル更新できるか` の評価が困難。Rust 実装によって、
   現実的な CPU 性能上限を計測する。
2. **bit-exact なオラクル**: 既存の MATLAB リファレンス（`vi_full_reference`）
   および C リファレンス（`host/src/vi_reference_c.c`）と完全に一致する
   Rust 実装を CI 上のリグレッションオラクルとして提供する。
3. **アルゴリズム特性の再評価**: MATLAB ベンチで観察された frontier / block /
   pyramid 系の特性（収束イテレーション数、updates 数、近似品質）を
   Rust で再現し、より大規模な map で計測可能にする。

### 1.2 スコープ

**移植対象（A + C）**:

- `vi_matlab/src/common/`: 全モジュール（`vi_params`, `unpack_transitions`,
  `coerce_transition_model`, `make_goal_mask`）
- `vi_matlab/src/cpu/reference/`: `vi_full_reference`,
  `compute_action_table_reference`
- `vi_matlab/src/cpu/frontier/`: 全 7 モジュール（`vi_frontier_bellman`,
  `vi_frontier_max_displacement`, `vi_frontier_2d`, `vi_frontier_3d`,
  `vi_frontier_3d_coarse_theta`, `vi_frontier_3d_tau`,
  `vi_frontier_3d_topk`, `vi_frontier_stack`）
- `vi_matlab/src/cpu/block/`: `vi_block_refine`, `vi_pyramid_sweep`
- `vi_matlab/src/fpga/stream/`: 全 6 モジュール（`vi_sweep_stream_algo`,
  `stream_strip_algo`, `compute_row_algo`, `load_row_algo`,
  `store_row_algo`, `cost_of`）
- `vi_matlab/src/shared/bitboard/`: 全 18 モジュール
- `vi_matlab/workflows/validation/tests/`: `gen_test_map`, `gen_transitions`
- `vi_matlab/workflows/benchmarks/`: `benchmark_vi`, `bench_cases`（CLI として再実装）

**スコープ外**:

- `vi_matlab/src/fpga/soc/vi_sweep_soc_kernel.m`（HDL Coder ターゲットの
  サイクルステップ SoC モデル。Rust に移植する意味が薄いため除外）
- 既存の `vi_rs/vi_ml/` ディレクトリ（用途未定。今回は触らない）
- MATLAB Coder MEX や codegen 関連
- HDL Verifier / cosim 関連

### 1.3 設計の根拠となる主要な選択

| 項目 | 採用 | 理由 |
|------|------|------|
| 一致目標 | bit-exact + Rust ネイティブ最適化 | MATLAB の `floor(accum / PROB_BASE)` と `c >= MV` クランプは Rust の `u32` 整数演算で同じビット列を 100× 以上速く生成できる |
| ベンチ | criterion + 自前 summary CLI | criterion はマイクロベンチ、summary CLI は `benchmark_vi.m` 互換のマクロ比較表 |
| 並列化 | serial デフォルト + `parallel` feature | serial はオラクル、parallel は実用最適化版 |
| API | Context struct + Solver trait | 横並びベンチで `Vec<Box<dyn Solver>>` を扱える、long-arg 関数のアンチパターン回避 |
| クレート分割 | 4 クレート（core / algorithm / fixtures / bench） | bitboard は frontier 専用のため独立クレート化しない（YAGNI） |

## 2. ワークスペース構成

```
vi_rs/
├── Cargo.toml                  # [workspace]
├── vi_core/                    # 不変データ契約（HLS / C リファレンス互換）
├── vi_algorithm/               # Solver trait + bitboard module + 全アルゴリズム
├── vi_fixtures/                # gen_test_map / gen_transitions
├── vi_bench/                   # criterion benches + summary CLI
└── vi_ml/                      # 既存。workspace に含めず残置
```

### 2.1 ワークスペース Cargo.toml

```toml
[workspace]
resolver = "2"
members = ["vi_core", "vi_algorithm", "vi_fixtures", "vi_bench"]

[workspace.package]
edition = "2021"
rust-version = "1.75"

[workspace.dependencies]
ndarray   = "0.16"
rayon     = "1.10"
thiserror = "1"
criterion = { version = "0.5", features = ["html_reports"] }
once_cell = "1"
rand      = "0.8"
proptest  = "1"
clap      = { version = "4", features = ["derive"] }
```

### 2.2 クレート間の依存

```
vi_core      ← vi_algorithm ← vi_fixtures ← vi_bench
                            ←─────────────────┘
```

- `vi_fixtures` および `vi_bench` から `vi_algorithm` を参照
- `vi_fixtures` の利用形態:
  - `vi_bench` からは **通常の `[dependencies]`**（`bench_summary` CLI が runtime で `generate_map` / `generate_transitions` を呼ぶ）
  - `vi_algorithm` からは **`[dev-dependencies]` のみ**（parity tests / proptest 等のテスト目的）
  - production code は `vi_core + vi_algorithm` のみで完結し、`vi_fixtures` は混入しない

## 3. `vi_core` クレート

### 3.1 責務

- HLS / C リファレンスと一致するデータ型と sentinel 定数
- `cost_of` の bit-exact 実装
- packed transitions と unpacked transition model の相互変換
- `make_goal_mask` 相当の goal-area マスク生成

アルゴリズムは一切含めない。`no_std` 化の余地を残す（当面は std 前提）。

### 3.2 モジュール

```
vi_core/src/
├── lib.rs              # re-exports
├── params.rs           # 定数（N_ACTIONS, N_THETA, PROB_BASE, ACTION_FW/ROT, ...）
├── types.rs            # Value, Penalty, Offset, ThetaIdx, ActionIdx, sentinels
├── cost.rs             # cost_of(): bit-exact
├── transitions.rs      # PackedTransitions ↔ TransitionModel
└── goal.rs             # GoalSpec, make_goal_mask
```

### 3.3 主要型と定数

```rust
pub type Value = u16;
pub type Penalty = u16;
pub type Offset = i8;
pub type ThetaIdx = u8;
pub type ActionIdx = u8;

pub const N_ACTIONS: usize = 6;
pub const N_THETA: usize = 60;
pub const MAX_VALUE: Value = 0xFFFF;
pub const PENALTY_OBSTACLE: Penalty = 0xFFFF;
pub const PENALTY_GOAL: Penalty = 0xFFFE;
pub const STEP_COST: u32 = 1;
pub const PROB_BASE: u32 = 262_144;
pub const MAX_OUTCOMES: usize = 10;
pub const TRANS_WORD_STRIDE: usize = 21;
pub const TRANS_TABLE_SIZE: usize = 7_560;

pub const ACTION_FW: [f64; N_ACTIONS] = [0.3, -0.2, 0.0, 0.2, 0.0, 0.2];
pub const ACTION_ROT: [f64; N_ACTIONS] = [0.0, 0.0, -20.0, -20.0, 20.0, 20.0];
```

### 3.4 `cost_of` の bit-exact 実装

```rust
#[inline]
pub fn cost_of(nv: Value, np_raw: Penalty) -> Value {
    if nv == MAX_VALUE || np_raw == PENALTY_OBSTACLE { return MAX_VALUE; }
    let np: u32 = if np_raw == PENALTY_GOAL { 0 } else { np_raw as u32 };
    let s = nv as u32 + np + STEP_COST;
    if s >= MAX_VALUE as u32 { MAX_VALUE - 1 } else { s as Value }
}
```

PENALTY_GOAL を隣接読み出し時に 0 とみなす規約はプロジェクト全体の load-bearing
不変条件。テストで明示的に確認する。

### 3.5 transitions モジュール

```rust
pub struct PackedTransitions(pub Vec<u32>);   // 長さ TRANS_TABLE_SIZE

pub struct TransitionModel {
    pub n_outcomes: [[u8; N_THETA]; N_ACTIONS],
    pub dix:  [[[Offset; MAX_OUTCOMES]; N_THETA]; N_ACTIONS],
    pub diy:  [[[Offset; MAX_OUTCOMES]; N_THETA]; N_ACTIONS],
    pub dit:  [[[Offset; MAX_OUTCOMES]; N_THETA]; N_ACTIONS],
    pub prob: [[[u32;    MAX_OUTCOMES]; N_THETA]; N_ACTIONS],
}

impl PackedTransitions {
    pub fn unpack(&self) -> TransitionModel;
}

impl TransitionModel {
    pub fn pack(&self) -> PackedTransitions;
    pub fn max_displacement(&self) -> (u8, u8, u8);  // |dix|, |diy|, |dit| の最大
}
```

`max_displacement` は frontier 系全変種が予測子コーンサイズを決めるために
使う。`vi_frontier_max_displacement.m` 相当。

### 3.6 goal モジュール

`make_goal_mask.m` を移植：

```rust
pub struct GoalSpec {
    pub xy_resolution: f64,
    pub map_origin_x: f64,
    pub map_origin_y: f64,
    pub goal_x: f64,
    pub goal_y: f64,
    pub goal_theta_deg: f64,
    pub goal_radius_m: f64,
    pub goal_margin_theta_deg: f64,
}

pub fn make_goal_mask(map_x: u32, map_y: u32, spec: &GoalSpec) -> Array3<bool>;
```

`Array3<bool>` は `ndarray::Array3<bool>` を返す（`vi_core` は ndarray を依存に含める例外。他には依存追加しない）。

## 4. `vi_algorithm` クレート

### 4.1 責務

- `Solver` trait と全アルゴリズム実装
- `VIContext` / `Budget` / `SolveStats` の共通インフラ
- bitboard モジュール（frontier 系専用ユーティリティ）
- frontier 系・block 系・stream 系・reference 系を統一 API で提供

### 4.2 モジュール構成

```
vi_algorithm/src/
├── lib.rs                          # Solver trait + 公開 API
├── context.rs                      # VIContext, Budget, SolveStats
├── bitboard/
│   ├── mod.rs                      # Bitboard2D / Bitboard3D 構造体
│   ├── ops.rs                      # popcount, dilate2d/3d, shift_row, ctz
│   ├── enumerate.rs                # enumerate2d/3d を Iterator として
│   └── conv.rs                     # from_logical / to_logical (ndarray 連携)
├── kernel/
│   ├── bellman.rs                  # 単セル Bellman backup（全 variant 共有）
│   └── norm.rs                     # top-k 用 prob_sum 正規化
├── reference/
│   ├── mod.rs                      # Reference solver
│   └── action_table.rs             # compute_action_table_reference 相当
├── frontier/
│   ├── mod.rs                      # 共通 frontier ループ skeleton
│   ├── f2d.rs                      # Frontier2D
│   ├── f3d.rs                      # Frontier3D
│   ├── stack.rs                    # FrontierStack
│   ├── coarse_theta.rs             # Frontier3DCoarseTheta
│   ├── tau.rs                      # Frontier3DTau
│   └── topk.rs                     # Frontier3DTopK
├── block/
│   ├── refine.rs                   # BlockRefine
│   └── pyramid.rs                  # PyramidSweep
└── stream/
    ├── mod.rs                      # StreamMimic
    ├── strip.rs                    # stream_strip_algo 相当
    ├── compute_row.rs              # compute_row_algo 相当
    └── load_store.rs               # load_row / store_row 相当
```

### 4.3 `VIContext`

```rust
pub struct MapDims { pub map_x: u32, pub map_y: u32 }

pub struct VIContext {
    pub dims: MapDims,
    pub value: ndarray::Array3<Value>,        // [map_y, map_x, N_THETA]
    pub penalty: ndarray::Array2<Penalty>,    // [map_y, map_x]
    pub goal_mask: ndarray::Array3<bool>,     // [map_y, map_x, N_THETA]
    pub transitions: TransitionModel,
}

impl VIContext {
    pub fn clone_value(&self) -> Self;  // ベンチで複数 solver に独立 value を渡すため
}
```

レイアウトは row-major（ndarray のデフォルト）。インデックス順 `(iy, ix, it)` は
MATLAB と HLS C リファレンスに揃える。

### 4.4 `Budget` と `SolveStats`

```rust
pub enum Budget {
    /// reference / block / pyramid 用（外側の sweep 数）
    Sweeps(u32),
    /// frontier 用（frontier 展開イテレーション数）
    Iterations(u32),
}

pub struct SolveStats {
    pub iters_or_sweeps: u32,
    pub updates: u64,
    pub final_delta: Value,
    pub converged: bool,
    pub extra: Option<SolveExtra>,  // pyramid 用 per-level stats など
}

pub enum SolveExtra {
    PyramidPerLevel(Vec<PyramidLevelStat>),
    ActionTable(ndarray::Array3<ActionIdx>),  // Reference のみ提供
}
```

### 4.5 `Solver` trait

```rust
pub trait Solver: Send + Sync {
    fn name(&self) -> &'static str;
    fn run(&self, ctx: &mut VIContext, budget: Budget) -> SolveStats;
}
```

各 variant は struct + `impl Solver`:

```rust
pub struct Reference { pub threshold: Value }
pub struct Frontier3D;
pub struct Frontier2D;
pub struct FrontierStack;
pub struct Frontier3DTau { pub tau: Value }
pub struct Frontier3DTopK { pub k: u32 }
pub struct Frontier3DCoarseTheta { pub coarse_step: u32, pub refine_iters: u32 }
pub struct BlockRefine {
    pub block_w: u32, pub block_h: u32,
    pub local_sweeps: u32, pub threshold: Value,
}
pub struct PyramidSweep {
    pub threshold: Value, pub min_size: u32,
    pub coarse_sweeps: u32, pub refine_sweeps: u32, pub descend_tau: Value,
}
pub struct StreamMimic;  // CU0 → CU1 を 1 sweep として多重実行
```

### 4.6 bitboard モジュール

```rust
pub struct Bitboard2D { /* data: Vec<u64>, dims */ }
pub struct Bitboard3D { /* data: Vec<u64>, dims */ }

impl Bitboard2D {
    pub fn new(map_x: u32, map_y: u32) -> Self;
    pub fn set(&mut self, ix: u32, iy: u32);
    pub fn test(&self, ix: u32, iy: u32) -> bool;
    pub fn popcount(&self) -> u64;
    pub fn dilate(&self, dx: u32, dy: u32) -> Self;
    pub fn and_inplace(&mut self, other: &Self);
    pub fn or_inplace(&mut self, other: &Self);
    pub fn complement(&self) -> Self;
    pub fn enumerate(&self) -> impl Iterator<Item = (u32, u32)>;
    pub fn from_logical(mask: ndarray::ArrayView2<bool>) -> Self;
    pub fn to_logical(&self) -> ndarray::Array2<bool>;
}

// Bitboard3D も同等。dilate には dt（θ 周期）を取る。
```

実装は uint64 ベース。`u64::count_ones()` / `u64::trailing_zeros()` を活用。

### 4.7 並列化

```toml
[features]
default = []
parallel = ["dep:rayon"]
```

- **serial（default）**: MATLAB と bit-exact 一致。
  - frontier 系は Gauss-Seidel 風 in-place 更新（MATLAB と同じ走査順序）
  - reference は MATLAB と同じ `for iy { for ix { for it { ... } } }`
- **parallel**: 内部で `#[cfg(feature = "parallel")]` 分岐
  - reference の sweep 内は二重バッファ（Jacobi 化）→ 収束 sweep 数は変わるが、
    収束点（fixed point）は同じ
  - frontier 系は enumerate した点列を `par_chunks` で並列 Bellman、
    結果を per-thread bitboard に書き出して OR で fold
  - bit-exact 性は serial 限定。parallel 時は `SolveStats.updates` や
    `iters_or_sweeps` は serial と異なってよい（収束 value table は一致）

### 4.8 bit-exact の担保

serial 実装の単体テストで以下を不変条件として確認：

- `Reference` の出力 == 既存 C リファレンス `host/src/vi_reference_c.c` の出力
  （手動 PR チェックリストで一度だけ確認）
- `Frontier3D / Frontier2D / FrontierStack / BlockRefine(threshold=0) /
  PyramidSweep` の出力 == `Reference` の出力（収束後の全要素一致）
- `Frontier3DTau / TopK / CoarseTheta` は近似変種なので
  「mean abs diff < tolerance」テストのみ

## 5. `vi_fixtures` クレート

### 5.1 責務

`gen_test_map.m` / `gen_transitions.m` の Rust 移植。テスト・ベンチ専用。

### 5.2 モジュール

```
vi_fixtures/src/
├── lib.rs
├── maps.rs
└── transitions.rs
```

### 5.3 API

```rust
pub enum MapType {
    Empty,
    Obstacle,
    Sentinel,
    Random { density: f64, seed: u64 },
}

pub struct GeneratedMap {
    pub value: ndarray::Array3<Value>,
    pub penalty: ndarray::Array2<Penalty>,
    pub goal_mask: ndarray::Array3<bool>,
    pub goal_x: u32,
    pub goal_y: u32,
    pub spec: GoalSpec,
}

pub fn generate_map(map_x: u32, map_y: u32, ty: MapType) -> GeneratedMap;

pub enum TransitionMode {
    Trivial,
    Full            { xy_resolution: f64 },
    PaperMonteCarlo { xy_resolution: f64 },
}

pub fn generate_transitions(mode: TransitionMode) -> PackedTransitions;
```

### 5.4 キャッシュ

`PaperMonteCarlo` は `64 × 64` サンプリングで重いため `once_cell::sync::Lazy`
で `(mode, xy_resolution)` キー → packed transitions をプロセス内 1 回のみ
キャッシュ。MATLAB の `persistent cache` と同等。

### 5.5 ランダム性

`rand::StdRng::seed_from_u64(seed)` を使う。MATLAB の Mersenne Twister
（`rng(seed)`）とは別実装なので、`Random` map の bit-exact 一致は諦める。

- MATLAB 一致テスト: `Empty / Obstacle / Sentinel` のみ
- `Random` map: Rust 内 reproducibility のみ保証（seed 固定で同一出力）

## 6. `vi_bench` クレート

### 6.1 責務

- criterion による各 solver のマイクロベンチ
- `benchmark_vi.m` 互換のマクロ比較 CLI（Markdown 表 + CSV）

### 6.2 ディレクトリ

```
vi_bench/
├── Cargo.toml
├── benches/
│   ├── reference.rs            # Reference solver
│   ├── frontier.rs             # Frontier3D/2D/Stack/Tau/TopK/CoarseTheta
│   ├── block_pyramid.rs        # BlockRefine / PyramidSweep
│   ├── stream_mimic.rs         # StreamMimic
│   └── bitboard.rs             # bitboard 内部 ops
└── src/
    └── bin/
        └── bench_summary.rs    # benchmark_vi.m 互換 CLI
```

### 6.3 criterion benches

各 `benches/*.rs` は `criterion_group!` で `{(map_type, size)} × {solvers}` の
組み合わせを `BenchmarkGroup` として登録。HTML レポート（`target/criterion/report/index.html`）に
solver ごとの throughput が並ぶ。

### 6.4 `bench_summary` CLI

```
cargo run -p vi_bench --release --bin bench_summary -- \
    --sizes 8,16,32,64 \
    --types empty,obstacle,sentinel,random \
    --max-sweeps 200 \
    --max-iters 4000 \
    --out vi_rs/target/bench_results/summary_<TS>.csv \
    --markdown \
    [--parallel]    # parallel feature で並列版を別行追加
```

- stdout に Markdown 表（`benchmark_vi.m::print_markdown_table` 互換）
- `--out` で CSV（`benchmark_vi.m::write_csv` の列順序を意識）
- exact variant のミスマッチを警告として stderr 出力、exit code に反映
- `--smoke` フラグで「各 solver が 1 case × 1 iter で動くか」のみ確認（CI 用）

### 6.5 引数パース

`clap` の derive マクロを利用。

## 7. テスト戦略

### 7.1 `vi_core` テスト

- `cost_of` 単体: PENALTY_GOAL → 0、obstacle、overflow クランプの 3 軸を網羅
- `PackedTransitions::unpack().pack() == self` のラウンドトリップ
- `make_goal_mask` を既知の小さい spec で MATLAB と数値一致比較
  （ハードコーディングしたゴールデン値）

### 7.2 `vi_algorithm` テスト

- `bitboard::*` 不変条件テスト（proptest）:
  - `to_logical(from_logical(m)) == m`
  - `popcount(dilate(b, 0, 0)) == popcount(b)`
  - `popcount(a | b) >= max(popcount(a), popcount(b))`
- 各 Solver の parity テスト（`Empty / Obstacle / Sentinel` × small size 5/8/16）:
  - exact variant: `assert_eq!(solver_output_value, reference_value)` 全要素
  - approximate variant: `mean_abs_diff(solver_value, reference_value) < tol`

### 7.3 C リファレンスとの cross-validation

既存 `host/src/vi_reference_c.c` と Rust `Reference` の出力一致を一度だけ
手動 PR チェックリストで確認。以降は Rust 内 parity tests で invariant を維持。

確認手順（チェックリストに記載）：

1. `vi_fixtures` で同一 `gen_test_map` 出力を `.bin` ダンプ
2. `host/vi_cli_mock --verify` 経由で C リファレンスを走らせ value table を `.bin` ダンプ
3. Rust `Reference` で同入力を走らせ value table を `.bin` ダンプ
4. `cmp` でバイナリ一致を確認

### 7.4 `vi_fixtures` テスト

- `generate_map` の決定性（seed 固定で同一出力）
- `PaperMonteCarlo` の不変量（合計確率 = `PROB_BASE * N_ACTIONS * N_THETA` 等）
- `Random` モードの seed-based reproducibility

### 7.5 `vi_bench` テスト

- `bench_summary --smoke` を CI で実行（各 solver が 1 case × 1 iter で動くか）

## 8. ビルドと Makefile 統合

ルート `Makefile` に以下のターゲットを追加：

```makefile
rs-test:
	cd vi_rs && cargo test --workspace

rs-bench:
	cd vi_rs && cargo bench -p vi_bench

rs-bench-summary:
	cd vi_rs && cargo run --release -p vi_bench --bin bench_summary -- \
	    --sizes 8,16,32,64 --types empty,obstacle,sentinel,random \
	    --markdown --out target/bench_results/summary_$(shell date +%Y%m%d_%H%M%S).csv

rs-bench-parallel:
	cd vi_rs && cargo run --release -p vi_bench --features parallel \
	    --bin bench_summary -- --parallel --markdown
```

## 9. 実装フェーズ（writing-plans で詳述）

以下の段階で実装する。各 Phase は独立にレビュー可能で、Phase 3 完了時点で
`Reference` solver は production 利用可能。

1. **Phase 1**: workspace + `vi_core`（型・cost_of・transitions・goal_mask）+ unit tests
2. **Phase 2**: `vi_algorithm` skeleton（VIContext / Budget / Solver trait / bitboard module）+ bitboard tests
3. **Phase 3**: `Reference` 実装 + MATLAB 既存テストケース移植
4. **Phase 4**: `Frontier3D / Frontier2D / FrontierStack`（bit-exact）+ parity tests
5. **Phase 5**: `Frontier3DTau / TopK / CoarseTheta`（近似）+ tolerance tests
6. **Phase 6**: `BlockRefine / PyramidSweep` + parity tests
7. **Phase 7**: `StreamMimic` + parity test
8. **Phase 8**: `vi_fixtures`（gen_test_map / gen_transitions）
9. **Phase 9**: `vi_bench`（criterion benches + bench_summary CLI）
10. **Phase 10**: `parallel` feature（rayon）追加 + 並列ベンチ

## 10. 未決事項

なし。設計に必要な質問は全て確認済み。

## 11. 受け入れ条件

- [ ] `cargo test --workspace` がパスする
- [ ] `cargo bench -p vi_bench` が完走し HTML レポートを出力する
- [ ] `cargo run --release -p vi_bench --bin bench_summary --` が
      `benchmark_vi.m` と同じケース行列（`Empty/Obstacle/Sentinel/Random × 8/16/32/64`）を
      実行し、Markdown 表と CSV を出す
- [ ] exact variants（Reference / Frontier{3D,2D,Stack} / BlockRefine(threshold=0) /
      PyramidSweep）の value mismatch が `Empty/Obstacle/Sentinel` 全ケースで 0
- [ ] approximate variants（Frontier3DTau / TopK / CoarseTheta）の
      mean abs diff が許容閾値以下
- [ ] `cargo run --release --features parallel ... -- --parallel` で
      並列ベンチが完走し、serial vs parallel の time/mismatch を出力
- [ ] C リファレンス `host/src/vi_reference_c.c` と Rust `Reference` の
      手動 cross-validation が完了している（PR チェックリストにレコード）
