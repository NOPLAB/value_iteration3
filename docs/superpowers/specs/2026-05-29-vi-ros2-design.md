# vi_ros2 設計仕様

- 起票日: 2026-05-29
- 関連: [vi_rs algorithm port](2026-05-22-vi-rs-algorithm-port-design.md)、`../value_iteration` (ROS1 catkin パッケージ)

## 1. 目的とスコープ

`vi_rs` (Rust 製の value iteration ライブラリ群) を、`ros2_rust/ros2_rust` を介して ROS2 Humble 上の Rust ノードから呼び出せるようにする。
インターフェース (アクション / トピック / パラメータの形と意味) は ROS1 の `../value_iteration` パッケージと **等価** にし、`vi_controller.py` 相当の Goal 送信フローや rviz による可視化フローがほぼそのまま使える状態を目指す。

**含む:**
- Action サーバ (`Vi.action` 相当) によるゴール受領・収束・キャンセル
- `nav_msgs/OccupancyGrid` (`/map`) からの penalty フィールド構築 (safety_radius 膨張込み)
- `value_function` / `policy` を `nav_msgs/OccupancyGrid` として transient_local publish
- online 走行 (`tf2` で現在姿勢を引き、`cmd_vel` を 10 Hz publish)
- `solver` パラメータによる vi_rs ソルバ切替 (`reference` / `frontier3d` / `block_refine` / `pyramid` ...)
- `parallel` Cargo feature による rayon 並列スイープ (value_iteration の `thread_num: 8` 相当)

**含まない:**
- LaserScan 駆動の local 再計画 (`ValueIteratorLocal::setLocalCost`)
- `grid_map_msgs::GetGridMap` 互換サービス
- Ultra96 (ARM64) cross-compile
- ROS1 ⇄ ROS2 ブリッジ

**互換性レベル**: ユーザ確認済み「インターフェース等価」。
ROS1 / ROS2 はメッセージ ABI が異なるため、ノード名・アクション名・パラメータキー・action 中身の意味論を揃えるが、メッセージ型は ROS2 流儀で再定義する。

## 2. アーキテクチャ概要

```
┌────────────────────────────────────────────────────────┐
│ vi_ros2/                                                │
│                                                         │
│  ┌────────────────────┐    ┌──────────────────────────┐ │
│  │ vi_interfaces      │    │ vi_node                  │ │
│  │ (ament_cmake)      │    │ (cargo-ament-build)      │ │
│  │                    │    │                          │ │
│  │ action/Vi.action   │◀───│ rclrs ノード             │ │
│  └────────────────────┘    │ ├── bridge.rs            │ │
│                            │ │   OG ↔ Penalty / ...   │ │
│         path = "../../vi_rs/..."                       │ │
│                            │ ├── solver_factory.rs    │ │
│                            │ │   "frontier3d" → Box…  │ │
│                            │ └── sweep_thread.rs      │ │
│                            │     std::thread + cancel │ │
│                            └──────────────────────────┘ │
│                                       │                 │
│              依存                     ▼                 │
│  ┌─────────────────────────────────────────────────┐    │
│  │ vi_rs (既存 Cargo workspace、unchanged)         │    │
│  │ vi_core / vi_algorithm / vi_fixtures            │    │
│  └─────────────────────────────────────────────────┘    │
└────────────────────────────────────────────────────────┘
            │
            ▼
   ROS2 Humble + colcon-cargo + cargo-ament-build
```

### 2.1 レイヤ責務

- **`vi_interfaces`** — `Vi.action` のみ定義する ament_cmake パッケージ。
  `rosidl_generator_rs` が rclrs から使える Rust 型を吐く。
- **`vi_node::bridge`** — ROS 型 ↔ vi_rs 型の純関数モジュール。
  `rclrs` 依存を一切持たず、`cargo test -p vi_node --lib` で単体テスト可能。
- **`vi_node::solver_factory`** — ROS パラメータ `solver: string` から `Box<dyn vi_algorithm::Solver>` を返す。
- **`vi_node::sweep_thread`** — `std::thread::spawn` で `Solver::run` を反復実行、`AtomicBool` で cancel、`crossbeam_channel` で feedback を流す。
- **`vi_node::main`** — ROS パラメータ読込、マップ取得、Action サーバ、value_function publisher、cmd_vel 配信 (online 時)。

### 2.2 外部 ROS インターフェース

| 方向 | 名前 | 型 | 備考 |
|---|---|---|---|
| Action server | `vi_controller` | `vi_interfaces/action/Vi` | value_iteration 互換 |
| Sub | `map` | `nav_msgs/OccupancyGrid` | transient_local QoS、起動時 1 回 latch 受信 |
| Pub | `value_function` | `nav_msgs/OccupancyGrid` | transient_local、現在姿勢 θ スライス |
| Pub | `policy` | `nav_msgs/OccupancyGrid` | transient_local、optimal action id を可視化 |
| Pub | `cmd_vel` | `geometry_msgs/Twist` | online 時のみ、10 Hz |
| Sub | `goal_pose` | `geometry_msgs/PoseStamped` | rviz "2D Goal Pose" → 内部 action client 起動 (オプション、`auto_goal: bool`) |
| TF | `map` → `base_link` | tf2 | online 時のみ |

## 3. コンポーネント詳細

### 3.1 `vi_interfaces`

```
vi_ros2/vi_interfaces/
├── package.xml          # ament_cmake, build_depend rosidl_default_generators
├── CMakeLists.txt       # rosidl_generate_interfaces + DEPENDENCIES geometry_msgs std_msgs
└── action/
    └── Vi.action
```

`Vi.action` (value_iteration と bit-同形):

```
geometry_msgs/PoseStamped goal
---
bool finished
---
std_msgs/UInt32MultiArray current_sweep_times
std_msgs/Float32MultiArray deltas
```

`current_sweep_times.data` と `deltas.data` は `len() == 1` で publish する (vi_rs は単一ソルバ、value_iteration の thread 別配列ではない)。
互換クライアントが iterate しても 1 要素見えるだけ。

### 3.2 `vi_node/Cargo.toml` 抜粋

```toml
[package]
name = "vi_node"
version = "0.1.0"
edition = "2021"

[[bin]]
name = "vi_node"
path = "src/main.rs"

[dependencies]
rclrs              = "*"     # workspace 提供
vi_interfaces      = "*"     # rosidl_generator_rs 生成、colcon 経由
std_msgs           = "*"
nav_msgs           = "*"
geometry_msgs      = "*"
vi_core            = { path = "../../vi_rs/vi_core" }
vi_algorithm       = { path = "../../vi_rs/vi_algorithm" }
vi_fixtures        = { path = "../../vi_rs/vi_fixtures" }
ndarray            = "0.16"
crossbeam-channel  = "0.5"
anyhow             = "1"

[features]
default  = ["parallel"]
online   = []                                # 将来 tf2_rs 追加時に有効化
parallel = ["vi_algorithm/parallel"]

# vi_rs ワークスペースから分離するため明示
[workspace]
```

`vi_algorithm/parallel` は `vi_algorithm/Cargo.toml:14` で定義され、rayon を有効化して `Reference::run()` / `Frontier3D::run()` を Jacobi 並列実装 (`run_parallel`) に dispatch する。
直列版 (Gauss-Seidel) と並列版 (Jacobi) は **同じ fixed point に収束する** ことが vi_rs のテスト (`parallel_converges_to_same_fixed_point_as_serial_*`) で保証されている。

`default = ["parallel"]` を選ぶ理由: value_iteration の常用 `thread_num: 8` と挙動を揃える。
直列モードが必要な場合 (オラクル等価テスト等) は `--no-default-features` でビルドする。

### 3.3 `bridge.rs` (ROS フリー)

純関数 API。`OccupancyGridView` / `PoseView` は ROS メッセージ依存を切り離すための薄い借用ラッパ:

```rust
pub struct OccupancyGridView<'a> {
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub data: &'a [i8],
}

pub struct PoseView {
    pub x: f64,
    pub y: f64,
    pub yaw_rad: f64,
}

pub struct PenaltyParams {
    pub safety_radius_m: f64,
    pub safety_radius_penalty: u16,
}

/// OccupancyGrid (-1, 0..100) → penalty 二次元配列。
///   data == 100 (or unknown=-1) → PENALTY_OBSTACLE
///   free → 0、加えて safety_radius 内の cell は safety_radius_penalty
pub fn occupancy_to_penalty(
    grid: &OccupancyGridView,
    params: &PenaltyParams,
) -> Array2<Penalty>;

/// PoseStamped → GoalSpec (theta wrapped to [0, 360))
pub fn pose_to_goal_spec(
    pose: &PoseView,
    grid: &OccupancyGridView,
    goal_radius_m: f64,
    goal_margin_theta_deg: f64,
) -> GoalSpec;

/// 現在姿勢の θ-slice を 0..100 に正規化して OccupancyGrid data 化
pub fn value_slice_to_occupancy(
    value: ArrayView3<Value>,
    theta_idx: usize,
    threshold: u8,
) -> Vec<i8>;

/// optimal action テーブル → OccupancyGrid (action id を 0..5 で塗る)
pub fn action_table_to_occupancy(
    actions: ArrayView3<ActionIdx>,
    theta_idx: usize,
) -> Vec<i8>;
```

`main.rs` 側で `nav_msgs::msg::OccupancyGrid` から `OccupancyGridView` を詰める。
これで `bridge.rs` は ROS 依存ゼロ → 純粋な `cargo test` 可能。

### 3.4 `solver_factory.rs`

```rust
pub fn make_solver(name: &str) -> Result<Box<dyn Solver>> {
    match name {
        "reference"       => Ok(Box::new(Reference::default())),
        "frontier3d"      => Ok(Box::new(Frontier3D::default())),
        "frontier3d_topk" => Ok(Box::new(Frontier3DTopK::default())),
        "frontier3d_tau"  => Ok(Box::new(Frontier3DTau::default())),
        "frontier3d_coarse_theta" => Ok(Box::new(Frontier3DCoarseTheta::default())),
        "frontier2d"      => Ok(Box::new(Frontier2D::default())),
        "frontier_stack"  => Ok(Box::new(FrontierStack::default())),
        "block_refine"    => Ok(Box::new(BlockRefine::default())),
        "pyramid"         => Ok(Box::new(PyramidSweep::default())),
        other             => Err(anyhow!("unknown solver: {other}")),
    }
}
```

### 3.5 `sweep_thread.rs`

```rust
pub struct SweepHandle {
    pub cancel: Arc<AtomicBool>,
    pub feedback_rx: Receiver<FeedbackTick>,
    pub request_tx: Sender<WorkerRequest>,
    pub join: JoinHandle<SolveStats>,
}

pub struct FeedbackTick {
    pub sweep_count: u32,
    pub final_delta: u16,
}

/// reader (publisher timer / cmd_vel timer) が worker に投げる読み取りリクエスト。
/// worker は Budget::Sweeps(1) と次の Sweeps(1) の間で drain して応答する。
pub enum WorkerRequest {
    ValueSlice    { theta_idx: usize,                resp: Sender<Array2<Value>> },
    OptimalAction { ix: i32, iy: i32, it: usize,     resp: Sender<ActionIdx> },
}

pub fn spawn_sweep(
    ctx: VIContext,
    solver: Box<dyn Solver>,
    cancel: Arc<AtomicBool>,
) -> SweepHandle;
```

**実装方針:**
- `Solver::run` は同期 blocking、進捗コールバックを持たない。
- ワーカースレッドは ctx を排他所有する (Mutex 不使用)。reader は WorkerRequest 経由で読む。
- ワーカースレッドは **`Solver::run(&mut ctx, Budget::Sweeps(1))` を反復呼び出し**、1 反復ごとに `FeedbackTick` を送り、リクエストを drain する:
  ```rust
  let mut total = 0u32;
  let mut last_stats: SolveStats;
  loop {
      // 読み取りリクエストを drain
      while let Ok(req) = request_rx.try_recv() {
          match req {
              WorkerRequest::ValueSlice { theta_idx, resp } => {
                  let slice = ctx.value.slice(s![.., .., theta_idx]).to_owned();
                  let _ = resp.send(slice);
              }
              WorkerRequest::OptimalAction { ix, iy, it, resp } => {
                  let aid = vi_algorithm::policy::optimal_action_at(&ctx, ix, iy, it);
                  let _ = resp.send(aid);
              }
          }
      }
      if cancel.load(Ordering::Relaxed) { break; }
      let stats = solver.run(&mut ctx, Budget::Sweeps(1));
      total += stats.iters_or_sweeps;
      last_stats = stats;
      let _ = feedback_tx.send(FeedbackTick {
          sweep_count: total,
          final_delta: stats.final_delta,
      });
      if stats.converged { break; }
  }
  last_stats
  ```
- これは value_iteration の `valueIterationWorker(INT_MAX, t)` ＋ `finished()` ポーリングと等価。
- 「sweep 1 単位」の意味はソルバごとに異なる (Reference は全走査、Frontier3D は queue 1 反復) が、`Budget::Sweeps(1)` がソルバ実装側で「1 単位」として定義されている前提に乗る (vi_rs 既存契約)。
- **応答遅延**: reader リクエストは sweep 境界でしか drain されない。Reference + 大マップ (14000×800×60) で 1 sweep が 1–2 秒かかる場合、cmd_vel/value_function は最大 1–2 秒古くなる。value_iteration の挙動と同等で、value function は cmd_vel rate より緩やかに変化するので問題ない。

### 3.6 `main.rs` フロー

```
1. rclrs::init
2. Node 作成、パラメータ宣言
   - solver: "frontier3d"
   - theta_cell_num: 60                 ← assert == 60
   - safety_radius: 0.2
   - safety_radius_penalty: 30
   - goal_margin_radius: 0.3
   - goal_margin_theta: 15
   - action_list: list of {name, fw_m, rot_deg}  ← assert len == 6
   - online: bool
   - cost_drawing_threshold: 60
   - delta_threshold: u16
   - thread_num: 0                      ← 0 = rayon デフォルト (= num_cpus)
   - map_wait_sec: 30
   - auto_goal: bool                    ← /goal_pose 自動取り込み
3. rayon thread pool 初期化 (parallel feature on かつ thread_num > 0 のみ)
4. /map を transient_local で待ち受け、1 メッセージ受信
5. bridge::occupancy_to_penalty で penalty 構築
6. vi_fixtures::generate_transitions(TransitionMode::PaperMonteCarlo { xy_resolution }) で transitions
7. VIContext を初期化 (value = MAX_VALUE で埋める、goal_mask = all-false)
8. Action server "vi_controller" 登録
   GoalCallback:
     a. bridge::pose_to_goal_spec → make_goal_mask で goal_mask 更新
     b. ctx.value を MAX_VALUE で再初期化、goal_mask が true のセルを 0 にピン留め
     c. spawn_sweep(ctx_clone, solver_clone, cancel)
     d. 10 Hz で feedback を pump
9. value_function / policy refresh timer (1 Hz):
     handle.request_tx に WorkerRequest::ValueSlice { theta_idx, resp } を送り、応答 slice を publish
10. online タイマ (10 Hz):
     tf lookup → handle.request_tx に WorkerRequest::OptimalAction { ix, iy, it, resp } を送り、応答 action_id から cmd_vel 生成
```

### 3.7 `launch/vi_navigation.launch.py`

```python
Node(
    package='vi_node',
    executable='vi_node',
    name='vi_node',
    parameters=[{
        'solver': 'frontier3d',
        'theta_cell_num': 60,
        'safety_radius': 0.2,
        'safety_radius_penalty': 30,
        'goal_margin_radius': 0.3,
        'goal_margin_theta': 15,
        'online': False,
        'cost_drawing_threshold': 60,
        'delta_threshold': 0,
        'thread_num': 0,
        'action_list': [
            {'name': 'forward',  'onestep_forward_m':  0.3, 'onestep_rotation_deg':   0.0},
            {'name': 'back',     'onestep_forward_m': -0.2, 'onestep_rotation_deg':   0.0},
            {'name': 'right',    'onestep_forward_m':  0.0, 'onestep_rotation_deg': -20.0},
            {'name': 'rightfw',  'onestep_forward_m':  0.2, 'onestep_rotation_deg': -20.0},
            {'name': 'left',     'onestep_forward_m':  0.0, 'onestep_rotation_deg':  20.0},
            {'name': 'leftfw',   'onestep_forward_m':  0.2, 'onestep_rotation_deg':  20.0},
        ],
    }],
)
```

## 4. データフロー & 状態遷移

### 4.1 起動シーケンス

```
main
 ├─ rclrs::init
 ├─ Node 生成 + パラメータ宣言
 ├─ パラメータ検証
 │   ├─ theta_cell_num == 60        else panic
 │   ├─ action_list.len() == 6      else panic
 │   ├─ action_list[i] が ACTION_FW[i]/ACTION_ROT[i] と一致  else panic (allow_action_mismatch=true で warn)
 │   └─ solver name が make_solver で解決可          else panic
 ├─ rayon thread pool 初期化 (parallel feature + thread_num > 0)
 ├─ map 受信 (transient_local subscriber、blocking until first msg)
 ├─ MapResources 構築:
 │   ├─ penalty:     bridge::occupancy_to_penalty(...)
 │   ├─ transitions: generate_transitions(PaperMonteCarlo { xy_resolution })
 │   └─ dims:        MapDims { map_x, map_y }
 ├─ Action server "vi_controller" 登録
 ├─ value_function / policy publisher 登録 (transient_local)
 ├─ value_function refresh timer (1 Hz) 登録
 ├─ if online {
 │     tf2 buffer + listener 構築
 │     cmd_vel publisher 登録
 │     cmd_vel timer (10 Hz) 登録
 │   }
 └─ rclrs::spin()
```

### 4.2 ゴール受領 → 収束までの状態遷移

```
                                          ┌─────────────┐
                                          │   Idle      │
                                          └──────┬──────┘
                                                 │ goal_callback(PoseStamped)
                                                 ▼
                              ┌────────────────────────────────┐
                              │ Building VIContext              │
                              │  ├─ value = MAX_VALUE で再初期化│
                              │  ├─ bridge::pose_to_goal_spec   │
                              │  ├─ make_goal_mask              │
                              │  └─ value[goal_mask] = 0        │
                              └────────────┬───────────────────┘
                                           │
                                           ▼
                              ┌────────────────────────────────┐
                              │ Sweeping (worker thread)        │
                              │  ├─ Budget::Sweeps(1) を反復    │
                              │  ├─ feedback tick ごとに        │
                              │  │   publish_feedback           │
                              │  └─ cancel || converged で抜ける│
                              └────────────┬───────────────────┘
                                           │
                          ┌────────────────┼────────────────┐
                  cancel │                 │ converged      │ budget_exhausted
                         ▼                 ▼                ▼
                  Result{finished:false}  Result{finished:true} Result{finished:false}
                         │                 │                │
                         └────────────────▼┴────────────────┘
                                          │
                                          ▼
                                   ┌──────────────┐
                                   │   Idle       │
                                   └──────────────┘
```

`Sweeping` 中に新規ゴールが届いたら **現在の worker を cancel → join → 新規開始**。
同時実行ゴールは禁止 (Action server の `goal_callback` で preempt)。

**コンパイル時 feature による挙動差:**

| feature | `Solver::run(Budget::Sweeps(1))` の中身 | スループット | 1 反復あたりの収束量 |
|---|---|---|---|
| なし | Gauss-Seidel (in-place 行 0..my スイープ) | シングルコア | 多い (in-place) |
| `parallel` | Jacobi (double-buffer、rayon 行並列) | マルチコア | 少なめ (反復回数増) |

両者は同じ fixed point に収束する。`bit-exact` 等価ではなく `same fixed point` 等価。

### 4.3 value_function / policy publish (1 Hz)

`ctx.value` は worker が排他所有する。`Arc<Mutex<VIContext>>` は使わず、`WorkerRequest::ValueSlice` 経由で **theta-slice snapshot** を取り出す方式:

```rust
// publisher timer:
let theta_idx = yaw_to_theta_idx(current_yaw);  // !online なら yaw = 0
let (resp_tx, resp_rx) = bounded(1);
if handle.request_tx.send(WorkerRequest::ValueSlice { theta_idx, resp: resp_tx }).is_err() {
    return;   // worker 終了済み
}
let slice = match resp_rx.recv_timeout(Duration::from_millis(2000)) {
    Ok(s)  => s,
    Err(_) => return,   // 大マップで sweep 進行中、次のチック待ち
};
let data = bridge::value_slice_to_occupancy(slice.view().insert_axis(Axis(2)), 0, threshold);
publisher.publish(OccupancyGrid { ..map_meta, data });
```

ポイント:
- `Array3<Value>` 全体は `1400 × 800 × 60 × 2 byte ≈ 130 MB` で、1 Hz でも全 clone は重い (とくに Ultra96)。slice のみ (2.2 MB) なら現実的。
- worker は `Budget::Sweeps(1)` 境界で `request_rx.try_recv()` を drain する (§3.5)。
- `!online` のときは `current_yaw = 0` 固定 (value_iteration の `vi_node.cpp` も online=false なら `yaw_ = 0` のまま)。
- タイムアウト 2 秒は Reference + 大マップで 1 sweep に要する最悪時間を上回るマージン。タイムアウト時はそのチックをスキップ。

### 4.4 cmd_vel (online、10 Hz)

```rust
// cmd_vel timer:
let (x, y, yaw) = match tf_lookup("map", "base_link", now) {
    Ok(t)  => t,
    Err(_) => { publish(Twist::zero()); return; }
};
let (ix, iy, it) = pose_to_cell(x, y, yaw, &map_meta);

let (resp_tx, resp_rx) = bounded(1);
if handle.request_tx.send(WorkerRequest::OptimalAction { ix, iy, it, resp: resp_tx }).is_err() {
    publish(Twist::zero()); return;
}
let action_id = match resp_rx.recv_timeout(Duration::from_millis(2000)) {
    Ok(a)  => a,
    Err(_) => { publish(Twist::zero()); return; }   // worker 忙し、次チック待ち
};
let twist = action_to_twist(&action_list[action_id as usize], dt);
publish(twist);
```

最適アクションの計算は worker 側で `vi_algorithm::policy::optimal_action_at(&ctx, ix, iy, it) -> ActionIdx` を呼ぶ (§3.5 WorkerRequest::OptimalAction 経由)。
既存 `compute_bellman` の選択ロジックを 1 セル分だけ抽出した形。
**この helper は vi_algorithm に追加する小さい API 変更** であり、本仕様の付随作業。

レスポンスタイムアウト時は zero Twist で fallback。10 Hz timer が継続するので、次の sweep 境界で再試行される。

### 4.5 ゴール座標 → 内部 (ix, iy) 変換

```rust
let ix = ((goal.x - map.origin.x) / map.resolution).floor() as i32;
let iy = ((goal.y - map.origin.y) / map.resolution).floor() as i32;
```

`map.origin.theta` は 0 前提 (value_iteration も同じ前提)。
非 0 の場合は warn を 1 回出して 0 として扱う。

## 5. エラーハンドリング

### 5.1 起動時 (fail-fast)

| 条件 | 振る舞い |
|---|---|
| `theta_cell_num != 60` | `panic!` 「vi_rs is compiled with N_THETA=60」 |
| `action_list.len() != 6` | `panic!` 「vi_rs requires exactly 6 actions」 |
| `solver` 文字列未対応 | `panic!` 候補リストを表示 |
| `action_list[i]` が `ACTION_FW[i]` / `ACTION_ROT[i]` と乖離 (> 1e-6) | デフォルト `panic!`、`allow_action_mismatch: true` で warn のみ |
| `map` トピックが `map_wait_sec` (default 30) 内に来ない | `panic!` |
| `map.info.width * height == 0` or `data.len()` 不一致 | `panic!` |
| OccupancyGrid セル値が `< -1` | `panic!` |

panic はプロセス終了。`ros2 launch` 側で `respawn=false` 推奨。

### 5.2 ランタイム (継続)

| 条件 | 振る舞い |
|---|---|
| ゴール受領中に新ゴール | 旧 sweep cancel → join → 新規開始。新ゴール accept。 |
| Sweep スレッド panic | join で検知、Result `finished: false`、ノード継続。error log。 |
| `tf_lookup` 失敗 | online cmd_vel timer は zero Twist publish。value_function publisher は last good yaw を使う。warn を 1 Hz スロットル。 |
| `optimal_action` で全 action `MAX_VALUE` | action_id = 停止相当 (rot 0 / fw 0)。warn 出力。 |
| Publisher channel 飽和 | rclrs デフォルトで drop。明示的に history depth = 1。 |
| Cancel 要求 (preempt) | `cancel.store(true)`。worker は次の Sweep 境界で抜ける。最大遅延 = 1 sweep 時間。 |

### 5.3 ログ

`rclrs` ロガー、ノード名 `vi_node`。Sweep 1 単位ごとの delta log は `debug` (デフォルト非表示)。

### 5.4 シャットダウン

- `rclrs::spin` 抜けたら destructor で worker thread に cancel → join (最大 1 sweep 待ち)。
- ハング回避のため join に **5 秒タイムアウト**、超えたら detach + warn。

## 6. テスト戦略

### 6.1 `bridge.rs` 単体テスト (ROS フリー)

`vi_node/src/bridge.rs` の `#[cfg(test)] mod tests`、`cargo test -p vi_node --lib` で実行。

- `occupancy_to_penalty`:
  - 全 free → 全 0、obstacle セルが PENALTY_OBSTACLE
  - safety_radius 内のセルが penalty 値で塗られている (golden: 5x5 grid, 中心 obstacle, radius=1cell)
  - 未知 (-1) の扱い (デフォルト obstacle 扱い)
- `pose_to_goal_spec`:
  - yaw=0 → goal_theta_deg=0、yaw=π/2 → 90
  - yaw < 0 は `[0, 360)` に wrap
- `value_slice_to_occupancy`:
  - MAX_VALUE → -1、0 → 0、threshold 超え → 100
- `action_table_to_occupancy`:
  - action id 0–5 が等間隔値にマップ

### 6.2 `solver_factory.rs` 単体テスト

- 既知文字列が `Box<dyn Solver>` を返す
- `solver.name()` が文字列と一致
- 未知文字列が `Err`

### 6.3 `sweep_thread.rs` 単体テスト

- 小さい fixture map (`vi_fixtures::generate_map(8, 8, Empty)`) で:
  - spawn → 数 tick → converge → join → `stats.converged == true`
  - spawn → cancel.store(true) → join → `stats.converged == false`
  - 同時 spawn 2 つを cancel & join が race しない

### 6.4 統合テスト (rclrs あり)

`vi_node/tests/integration.rs`、`cargo test -p vi_node --test integration`。ROS2 環境必須、CI では skip マーク。

- 別プロセスで vi_node 起動
- rclrs client で `map` を publish、`vi_controller` action send_goal
- feedback の `current_sweep_times.data.len() == 1` を検証
- result の `finished == true` を検証
- small map ケース (20x20、empty、center goal、<1 秒で収束)

### 6.5 オラクル等価 (vi_rs Reference と bit-exact)

`tests/oracle_equivalence.rs`、`cargo test -p vi_node --test oracle_equivalence --no-default-features`。
`parallel` feature を切って **直列 Reference** と比較し、ROS layer の透明性 (vi_rs の値を変えていないこと) を確認。

```rust
let ctx_via_bridge = build_via_bridge(small_map);
let mut ctx_direct = ctx_via_bridge.clone_value();
Reference::default().run(&mut ctx_direct, Budget::Sweeps(200));
let mut ctx_bridge = ctx_via_bridge;
Reference::default().run(&mut ctx_bridge, Budget::Sweeps(200));
assert_eq!(ctx_bridge.value, ctx_direct.value);
```

### 6.6 value_iteration ROS1 等価 (手動)

ROS1 環境ありで手動テスト手順 (CI 不要、README に記述):

1. ROS1 `navigation_house.launch` で `value_function` を rosbag 記録
2. ROS2 `vi_node` で同マップ・同ゴールで `value_function` を rosbag2 記録
3. 2 つの OccupancyGrid を data[] 単位で diff、`|a - b| <= 1` で一致と判定

### 6.7 colcon ビルドテスト

`scripts/ros2_test.sh`:

```bash
. /opt/ros/humble/setup.bash
cd vi_ros2_ws
colcon build --packages-select vi_interfaces vi_node
colcon test --packages-select vi_node
```

## 7. Docker / CI / Makefile 統合

### 7.1 `vi_ros2/docker/Dockerfile`

```dockerfile
FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y --no-install-recommends \
      git curl build-essential pkg-config libclang-dev \
      python3-colcon-common-extensions \
      python3-colcon-cargo python3-colcon-ros-cargo \
      libssl-dev cmake \
    && rm -rf /var/lib/apt/lists/*

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
      sh -s -- -y --default-toolchain 1.75.0
ENV PATH=/root/.cargo/bin:$PATH

RUN cargo install --version 0.1.10 cargo-ament-build

WORKDIR /ros2_rust_ws/src
RUN git clone https://github.com/ros2-rust/ros2_rust.git && \
    vcs import < ros2_rust/ros2_rust_humble.repos

WORKDIR /ros2_rust_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --merge-install --packages-up-to rclrs

WORKDIR /workspace
CMD ["/bin/bash"]
```

### 7.2 ビルド手順 (コンテナ内)

```bash
. /opt/ros/humble/setup.bash
. /ros2_rust_ws/install/local_setup.bash

cd /workspace/value_iteration_fpga
mkdir -p vi_ros2_ws/src
ln -sf $(pwd)/vi_ros2/vi_interfaces vi_ros2_ws/src/
ln -sf $(pwd)/vi_ros2/vi_node       vi_ros2_ws/src/

cd vi_ros2_ws
# デフォルト (parallel feature on)
colcon build --packages-select vi_interfaces vi_node \
       --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/local_setup.bash

# シリアル (bit-exact 検証用)
colcon build --packages-select vi_node \
       --cmake-args -DCMAKE_BUILD_TYPE=Release \
       --cargo-args --no-default-features

# 実行
ros2 launch vi_node vi_navigation.launch.py
```

### 7.3 トップレベル `Makefile` 追加分

```makefile
VI_ROS2_DOCKER_IMG ?= vi_ros2_dev:humble

ros2-docker:
	docker build -t $(VI_ROS2_DOCKER_IMG) vi_ros2/docker

ros2-shell:
	docker run --rm -it \
	  -v $(PWD):/workspace/value_iteration_fpga \
	  -w /workspace/value_iteration_fpga \
	  $(VI_ROS2_DOCKER_IMG)

ros2-build:
	docker run --rm \
	  -v $(PWD):/workspace/value_iteration_fpga \
	  -w /workspace/value_iteration_fpga \
	  $(VI_ROS2_DOCKER_IMG) \
	  bash scripts/ros2_build.sh

ros2-test:
	docker run --rm \
	  -v $(PWD):/workspace/value_iteration_fpga \
	  -w /workspace/value_iteration_fpga \
	  $(VI_ROS2_DOCKER_IMG) \
	  bash scripts/ros2_test.sh
```

### 7.4 `.gitignore` 追加分

```
vi_ros2_ws/
vi_ros2/**/target/
vi_ros2/**/install/
vi_ros2/**/build/
vi_ros2/**/log/
```

### 7.5 Ultra96 ターゲット (スコープ外)

Ultra96-V2 (ARM64) で走らせる場合は ros2_rust の cross-compile が必要。
今回のスコープは **x86_64 開発ホストでの動作確認まで**。
Ultra96 デプロイは別フェーズ (Petalinux 統合と合わせて検討)。

## 8. オープン項目 / 将来の拡張

- LaserScan ベース local 再計画 (`ValueIteratorLocal::setLocalCost` 相当)
- vi_ml クレートが用意する ML ベースの経験値初期化を Action パラメータとして受領
- FPGA ハードウェアバックエンド (`libvi_sweep` を呼ぶ vi_node モード切替)
- ros2 launch / rviz の同梱コンフィグ
- Ultra96 ARM64 cross-compile + Petalinux イメージ統合
