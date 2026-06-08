# 本家 value_iteration 忠実移植 reference 設計仕様

- 起票日: 2026-06-08
- 対象: `../value_iteration` (本家 ROS1 catkin `value_iteration` パッケージ) の `ValueIterator` / `ValueIteratorLocal` を Rust に**忠実移植**する新クレート `vi_rs/vi_reference/`
- 関連: [vi_rs algorithm port](2026-05-22-vi-rs-algorithm-port-design.md)、[本家 vs vi_ros2 比較ベンチ](2026-06-04-vi-ros-compare-benchmark-design.md)

## 1. 背景と目的

`vi_rs/vi_algorithm` の既存 `Reference` ソルバーは **HLS / MATLAB ストリーミングカーネルの 16-bit データ契約** (`value_t = u16`, packed transitions, `goal_mask`, `cost_of = neighbor + penalty + STEP_COST(1)`) を bit-exact に再現したもので、FPGA 本番カーネルのオラクルとして機能する。これは本家 ROS1 C++ `ValueIterator` の**量子化版**であり、数値モデルが本家とは異なる。

本タスクの目的は、**本家 `ValueIterator` / `ValueIteratorLocal` の挙動を、型・アルゴリズム・固有バグまで含めて Rust で忠実再現**した独立の reference を新規に作ることである。これにより:

- 本家アルゴリズムの CPU 側「真の」オラクルを vi_rs ワークスペース内に持てる。
- 既存 16-bit 契約・FPGA オラクル・約 20 個のパリティテストは**一切変更しない**(無傷)。

**本家とは「忠実」= bit / 挙動レベルで一致**を意味する。最適化・簡約・バグ修正は行わない。

## 2. スコープ

### 含む
- 本家 `ValueIterator` 基底クラスのフルパイプライン:
  `setMapWithOccupancyGrid` / `setMapWithCostGrid` → `setState` → `setStateTransition` → `setSweepOrders` → `setGoal` / `setStateValues` → `valueIterationWorker` / `valueIteration` / `actionCost` → `posToAction` / `makeValueFunctionMap` / `valueFunctionWriter` / `policyWriter` / `finished` / status 系。
- 本家 `ValueIteratorLocal` 派生クラス: `setLocalCost` (LaserScan) / `localValueIterationWorker` / `localValueIterationLoop` / `valueIterationLocal` / `actionCostLocal` / `setLocalWindow` / `inLocalArea` / `posToAction` override / `makeLocalValueFunctionMap`。
- 本家のマルチスレッド構造 (`setStateTransition` のθ並列、`valueIterationWorker` の共有 `states_` 並行更新) の再現。
- 本家の**固有バグ・数値挙動**の再現 (§7, §11)。

### 含まない (非目標)
- ROS ランタイム (roscpp / actionlib / tf / service / `XmlRpc` パラメータ) の再現。アクション集合・ゴール・マップは Rust API 引数で与える。
- `grid_map` / `grid_map_msgs` への実シリアライズ。出力は数値内容が同一のプレーン Rust データで返す (§10)。
- 本家 `vi_node*.cpp` のノード制御ループ・RViz・localization。
- アルゴリズムの最適化・修正・並列化の改善。
- 既存 `vi_core` / `vi_algorithm` の変更。

## 3. 配置とクレート構成

新規ワークスペースメンバ `vi_rs/vi_reference/` を追加する。

```
vi_rs/
├─ Cargo.toml                # members に "vi_reference" を追加
└─ vi_reference/
   ├─ Cargo.toml             # 依存: std のみ (ndarray も vi_core も不要)
   └─ src/
      ├─ lib.rs              # 公開 re-export、定数
      ├─ params.rs           # 定数 (PROB_BASE 等)
      ├─ state_transition.rs # StateTransition
      ├─ action.rs           # Action
      ├─ state.rs            # State + 2 コンストラクタ
      ├─ sweep_status.rs     # SweepWorkerStatus
      ├─ msg.rs              # OccupancyGrid / Quaternion / LaserScan の最小代替型
      ├─ value_iterator.rs   # ValueIterator (基底) フルパイプライン
      └─ local.rs            # ValueIteratorLocal (基底を内包/合成)
```

- `vi_core` 非依存 (本家は独自の数値契約)。`std` のみ。`rand` 不要 (本家の遷移生成は決定的サブセルサンプリング)。
- **命名は本家に忠実**: 型は `ValueIterator` / `ValueIteratorLocal` / `State` / `Action` / `StateTransition` / `SweepWorkerStatus`。メソッドは Rust 規約に合わせ snake_case (`set_map_with_occupancy_grid` 等)。
- Rust では継承が無いため `ValueIteratorLocal` は `ValueIterator` を**フィールドとして内包** (`base: ValueIterator`) し、override メソッド (`pos_to_action`, `set_map_with_occupancy_grid`) を再定義、`states_` 等へは `base` 経由でアクセスする (合成 + 委譲)。

## 4. 型と定数 (本家忠実)

本家 `ValueIterator.h` 末尾の定数と完全一致させる。

| 本家 | Rust | 値 |
|---|---|---|
| `resolution_xy_bit_` (uchar) | `RESOLUTION_XY_BIT: u32` | 6 |
| `resolution_t_bit_` (uchar) | `RESOLUTION_T_BIT: u32` | 6 |
| `prob_base_bit_` (uchar) | `PROB_BASE_BIT: u32` | 18 (= 6*2+6) |
| `prob_base_` (uint64) | `PROB_BASE: u64` | 262144 (= 1<<18) |
| `max_cost_` (uint64) | `MAX_COST: u64` | 262_144_000_000_000 (= 1_000_000_000 * PROB_BASE) |

- コスト系 (`total_cost_`, `penalty_`, `local_penalty_`, `min_cost`, `max_delta`): **`u64`**。
- 座標・オフセット・prob・セル数 (`ix_`,`iy_`,`it_`,`dix`,`diy`,`dit`,`_prob`,`cell_num_*`): **`i32`** (本家 `int`)。
- 解像度・角度・ゴール座標 (`xy_resolution_`,`t_resolution_`,`map_origin_*`,`goal_x_`,`goal_y_`): **`f64`** (本家 `double`)。
  - 注: 本家 `t_resolution_` は `double` メンバだが `t_resolution_ = 360/cell_num_t_;` で `360`(int)/`cell_num_t_`(int) の**整数除算**後に `double` へ昇格 → `cell_num_t_=60` で `6.0`。**この整数除算を忠実再現** (`(360 / cell_num_t) as f64`、`cell_num_t=60` で `6.0`、例えば `cell_num_t=7` なら `51.0` になる丸め)。
- `goal_t_`, `goal_margin_theta_`: `i32`。
- `optimal_action_`: 本家 `Action*` → Rust では `Option<usize>` (`actions` ベクタの索引)。**唯一の型適応**であり、挙動 (どのアクションが選ばれたか、id / delta の読み出し) は同一。

## 5. データ構造

```rust
// state_transition.rs — 本家 StateTransition
pub struct StateTransition { pub dix: i32, pub diy: i32, pub dit: i32, pub prob: i32 }
impl StateTransition { pub fn to_string(&self) -> String { /* "dix:.. diy:.. dit:.. prob:.." */ } }

// action.rs — 本家 Action
pub struct Action {
    pub name: String,
    pub delta_fw: f64,   // _delta_fw [m]
    pub delta_rot: f64,  // _delta_rot [deg]
    pub id: i32,         // id_
    pub state_transitions: Vec<Vec<StateTransition>>, // [theta] -> 遷移リスト
}
impl Action { pub fn new(name: impl Into<String>, fw: f64, rot: f64, id: i32) -> Self }

// state.rs — 本家 State
pub struct State {
    pub total_cost: u64,         // total_cost_
    pub penalty: u64,            // penalty_
    pub local_penalty: u64,      // local_penalty_
    pub ix: i32, pub iy: i32, pub it: i32,
    pub free: bool,              // free_
    pub final_state: bool,       // final_state_
    pub optimal_action: Option<usize>, // optimal_action_ (Action* → 索引)
}
// 2 コンストラクタ (§7.6)。

// sweep_status.rs — 本家 SweepWorkerStatus
pub struct SweepWorkerStatus { pub finished: bool, pub sweep_step: i32, pub delta: f64 }
impl Default for SweepWorkerStatus { /* finished=false, sweep_step=0, delta = MAX_COST as f64 */ }
```

```rust
// value_iterator.rs
pub struct ValueIterator {
    pub states: Vec<State>,                    // states_
    pub actions: Vec<Action>,                  // actions_ (本家は参照、Rust は所有)
    pub sweep_orders: Vec<Vec<i32>>,           // sweep_orders_ (6 種)
    pub thread_status: BTreeMap<i32, SweepWorkerStatus>, // thread_status_ (id→status, 決定的順序のため BTreeMap)
    pub status: String,                        // status_
    pub goal_x: f64, pub goal_y: f64, pub goal_margin_radius: f64,
    pub goal_t: i32, pub goal_margin_theta: i32,
    pub thread_num: i32,
    pub xy_resolution: f64, pub t_resolution: f64,
    pub cell_num_x: i32, pub cell_num_y: i32, pub cell_num_t: i32,
    pub map_origin_x: f64, pub map_origin_y: f64,
    pub map_origin_quat: Quaternion,
}
```

## 6. 入力型 (ROS 型の最小代替) — `msg.rs`

本家が参照するフィールドのみを持つ最小構造体。

```rust
pub struct Quaternion { pub x: f64, pub y: f64, pub z: f64, pub w: f64 }

pub struct OccupancyGrid {
    pub width: i32,            // info.width
    pub height: i32,           // info.height
    pub resolution: f64,       // info.resolution
    pub origin_x: f64,         // info.origin.position.x
    pub origin_y: f64,         // info.origin.position.y
    pub origin_quat: Quaternion,
    pub data: Vec<i8>,         // data (ROS は int8: 0=free, それ以外=占有/unknown)
}

pub struct LaserScan {
    pub angle_min: f64,
    pub angle_increment: f64,
    pub ranges: Vec<f64>,
}
```

- 本家 `State` 構築は `map.data[idx] == 0` で free 判定し、`map.data[idx] != 0` で penalty を立てる。`setMapWithCostGrid` は `map.data[idx] & 0xFF` を `unsigned int cost` として使う。よって `data: Vec<i8>` を保持し、cost 版では `(data[idx] as u8) as u32` で `& 0xFF` 相当を取る。

## 7. アルゴリズム移植仕様 (メソッド単位・固有バグ明記)

各メソッドは本家 C++ と**行単位で対応**させる。以下、忠実再現で特に注意すべき点を列挙する。

### 7.1 `to_index` / `in_map_area`
- `to_index(ix,iy,it) = it + ix*cell_num_t + iy*(cell_num_t*cell_num_x)` (i32 演算、`states[to_index(..) as usize]`)。
- `in_map_area(ix,iy) = 0<=ix<cell_num_x && 0<=iy<cell_num_y`。

### 7.2 `cell_delta(x,y,t) -> (ix,iy,it)`
```
ix = floor(|x| / xy_resolution); if x<0 { ix = -ix - 1 }
iy = floor(|y| / xy_resolution); if y<0 { iy = -iy - 1 }
it = floor(t / t_resolution)
```
(`f64` 演算、結果 `i32`。`it` は **絶対**インデックス、負正規化しない。)

### 7.3 `no_noise_state_transition(a, from_x,from_y,from_t) -> (to_x,to_y,to_t)`
```
ang = from_t / 180 * PI
to_x = from_x + a.delta_fw*cos(ang)
to_y = from_y + a.delta_fw*sin(ang)
to_t = from_t + a.delta_rot
while to_t < 0.0 { to_t += 360.0 }   // ★固有挙動: 負しか正規化しない (>=360 は残す)
```

### 7.4 `set_state_transition_worker_sub(action, it)` — サブセルサンプリング
```
theta_origin = it * t_resolution               // f64
xy_sample_num = 1<<RESOLUTION_XY_BIT  = 64
t_sample_num  = 1<<RESOLUTION_T_BIT   = 64
xy_step = xy_resolution / xy_sample_num
t_step  = t_resolution  / t_sample_num
for oy in (0.5*xy_step .. xy_resolution step xy_step):   // 64 点
 for ox in (0.5*xy_step .. xy_resolution step xy_step):  // 64 点
  for ot in (0.5*t_step .. t_resolution step t_step):    // 64 点
    (dx,dy,dt) = no_noise_state_transition(a, ox, oy, ot + theta_origin)
    (dix,diy,dit) = cell_delta(dx,dy,dt)
    // 既存バケット (dix,diy,dit 一致) があれば prob++、無ければ push (dix,diy,dit,1)
```
- **★最重要の固有挙動**: `dix`,`diy` は原点セル(0,0)からの**変位 (delta)**だが、`dit = floor((θ_origin+ot+rot)/t_resolution)` は**絶対 θ インデックス**。x,y は相対、θ は絶対、という非対称を保持する。
- ループは f64 加算の累積。`0.5*step` から `< 上限` まで。**浮動小数の刻み (`for(double o=...; o<limit; o+=step)`) を忠実に**再現する。
- 合計サンプル数 = 64*64*64 = **262144 = PROB_BASE** → 1 アクションの全 prob 総和は 262144。`action_cost` の `>>18` (= /262144) と対応。

### 7.5 `set_state_transition` (θ並列)
- 本家: 各アクションの `_state_transitions` を `cell_num_t` 個の空リストで初期化後、`it=0..cell_num_t` の**θごとに 1 スレッド** (`setStateTransitionWorker`) を spawn し join。各スレッドは全アクションについて `set_state_transition_worker_sub(a, it)` を呼ぶ。
- 書き込み先 `a.state_transitions[it]` は θ ごとに**独立**なのでデータ競合なし → 結果は決定的。
- Rust 実装: θを `std::thread` で分割。各スレッドが `(action_idx, it)` の遷移リストを構築して返し、メイン側で `actions[a].state_transitions[it]` に格納する形が安全 (結果は本家と bit 一致)。

### 7.6 `State` 2 コンストラクタ
**occupancy 版** (`State(x,y,theta,map,margin,margin_penalty,x_num)`):
```
ix=x; iy=y; it=theta
total_cost = MAX_COST
penalty    = PROB_BASE
local_penalty = 0
final_state = false
optimal_action = None
free = (map.data[y*x_num + x] == 0)
if !free { return }
for ix2 in (-margin+x ..= margin+x):
 for iy2 in (-margin+y ..= margin+y):
   pos = iy2*x_num + ix2                     // i32/i64
   if 0 <= pos && pos < data.len() && map.data[(iy2*x_num + ix2)] != 0 {
       penalty = (margin_penalty * PROB_BASE as f64) as u64 + PROB_BASE
   }
```
- **★固有バグ**: 境界チェックが線形 `pos` のみで `ix2` の**列範囲を見ない**。`ix2` が負 / `>=x_num` でも `pos` が `[0,len)` に収まれば隣接行のセルを読む。`map.data[iy2*x_num+ix2]` は `data[pos]` と同一値。**この行跨ぎ参照を忠実再現** (Rust では `pos` を `i64` で計算し `0<=pos && (pos as usize)<data.len()` のときに `data[pos as usize]`)。
- `margin = ceil(safety_radius / xy_resolution)` を `i32`。
- `margin_penalty > 1.0e10` のとき本家は `ROS_ERROR` を出すだけ (計算は続行) → Rust では無視 or `eprintln!` 任意。

**cost 版** (`State(x,y,theta,cost)`):
```
ix=x; iy=y; it=theta
total_cost = MAX_COST
final_state = false
optimal_action = None
free = (cost != 255)
penalty = if free { (cost as u64) << PROB_BASE_BIT } else { 0 }
// 注: local_penalty はこのコンストラクタでは本家でセットしない (未初期化) → Rust は 0 で初期化 (安全側)。
//     setGoal→setStateValues で local_penalty=0 される経路を通るため実害なし。
```

### 7.7 `set_state` / `set_map_with_*`
- `setMapWithOccupancyGrid`: `cell_num_t/x/y`, `xy_resolution`, `t_resolution = (360/cell_num_t) as f64`, `map_origin_*`, `map_origin_quat` を設定 → `set_state` → `set_state_transition` → `set_sweep_orders`。
- `set_state`: `states.clear()`; `margin = ceil(safety_radius/xy_resolution)`; `for y { for x { for t { push State::occupancy(..) }}}`。
- `setMapWithCostGrid`: 同様だが `set_state` の代わりに inline で cost 版コンストラクタを使うループ、`margin` 計算は本家にあるが**未使用** (忠実に計算だけして捨てる)。

### 7.8 `set_sweep_orders` (6 種)
本家の 6 つの走査順を忠実生成:
- `[0]`: `y,x,t` 順 (toIndex)。
- `[1]`: `x,y,t` 順。
- `[2]`: `[0]` の逆順。
- `[3]`: `[1]` の逆順。
- `[4],[5]`: `half = sweep_orders[0].size()/2`。本家ループ:
  ```cpp
  for(int i=0;i<2;i++){
      sweep_orders_.push_back( {sweep_orders_[i].begin(), sweep_orders_[i].begin()+half} );
      sweep_orders_[4].insert(sweep_orders_[4].end(), sweep_orders_[i].begin()+half, sweep_orders_[i].end());
  }
  ```
  を逐語で追うと:
  - `i=0`: index4 に `[0]前半` を push → `[4]=[0]前半`。続けて `[4].append([0]後半)` → `[4]=[0]全体`。
  - `i=1`: index5 に `[1]前半` を push → `[5]=[1]前半`。続けて `[4].append([1]後半)` → `[4]=[0]全体 + [1]後半`。
  - **最終結果 (★固有のアンバランス/重複)**: `[4] = [0]全体 + [1]後半` (size = `[0].len() + half`)、`[5] = [1]前半` (size = `half`)。
  - この生成手順を**行単位で逐語移植**する (簡約して `[4]=後半結合` 等にしない)。
- 既に生成済み (`sweep_orders` 非空) なら何もしない。

### 7.9 `set_goal` / `set_state_values`
- `set_goal(x,y,t)`: `while t<0 {t+=360}; while t>=360 {t-=360}`; `goal_x/y/t` 設定; `thread_status.clear()`; `set_state_values()`; `status="calculating"`。
- `set_state_values`:
  - **距離判定**: セル左下 `(x0,y0)=(ix*xy_res+origin, iy*xy_res+origin)` と右上 `(x1,y1)=(x0+xy_res,y0+xy_res)` の両方がゴール半径内 (`r0<R^2 && r1<R^2`) かつ `free` → `final_state` 候補。
  - **向き判定**: `t0 = (it*t_resolution) as i32`, `t1 = ((it+1)*t_resolution) as i32` (f64→i32 切り捨て); `goal_t_2 = if goal_t>180 {goal_t-360} else {goal_t+360}`;
    `final_state &= (goal_t-margin_theta <= t0 && t1 <= goal_t+margin_theta) || (goal_t_2-margin_theta <= t0 && t1 <= goal_t_2+margin_theta)`。
  - 全 state: `total_cost = if final_state {0} else {MAX_COST}`; `local_penalty=0`; `optimal_action=None`。

### 7.10 `action_cost(s, a) -> u64` (★wrapping)
```
cost: u64 = 0
for tran in a.state_transitions[s.it as usize]:
    ix = s.ix + tran.dix; if ix<0 || ix>=cell_num_x { return MAX_COST }
    iy = s.iy + tran.diy; if iy<0 || iy>=cell_num_y { return MAX_COST }
    it = ((tran.dit + cell_num_t) % cell_num_t)        // ★ s.it を足さない。dit は絶対 θ
    after = states[to_index(ix,iy,it)]
    if !after.free { return MAX_COST }
    cost = cost.wrapping_add(
              (after.total_cost.wrapping_add(after.penalty).wrapping_add(after.local_penalty))
              .wrapping_mul(tran.prob as u64) )
return cost >> PROB_BASE_BIT
```
- **★固有挙動**: u64 オーバーフロー時の**折り返し**を `wrapping_*` で再現 (§11)。本家 unsigned 演算の折り返しは定義動作。
- いずれかの遷移先が「マップ外 or not free」なら即 `MAX_COST` 返却 (確率的に一部でも障害物に入るアクションは不可)。

### 7.11 `value_iteration(s) -> u64`
```
if !s.free || s.final_state { return 0 }
min_cost: u64 = MAX_COST; min_action: Option<usize> = None
for (idx,a) in actions:
    c = action_cost(s, a)            // u64
    if c < min_cost { min_cost=c; min_action=Some(idx) }
delta = (min_cost as i64) - (s.total_cost as i64)
s.total_cost = min_cost
s.optimal_action = min_action
return delta.unsigned_abs()           // |delta|
```
- 本家 `int64_t delta = min_cost - s.total_cost_;` は u64 減算→i64 再解釈。値域が i63 内なので `(i64)-(i64)` と等価。`return delta>0?delta:-delta` = `unsigned_abs`。

### 7.12 `value_iteration_worker(times, id)` (マルチスレッド・unsafe) / `finished`
- `thread_status.insert(id, SweepWorkerStatus::default())`。
- `for j in 0..times:`
  - `thread_status[id].sweep_step = j+1`
  - `max_delta: u64 = 0`
  - `for i in sweep_orders[(id as usize) % sweep_orders.len()]: max_delta = max(max_delta, value_iteration(&mut states[i]))`
  - `thread_status[id].delta = (max_delta >> PROB_BASE_BIT) as f64`   // ★二重シフト (報告用)
  - `if status=="canceled" || status=="goal" { break }`  // (`delta<0.1` 判定は本家でもコメントアウト)
- `thread_status[id].finished = true`
- **並行性**: 複数スレッドが共有 `states` を `sweep_orders[id%6]` の順に Gauss-Seidel 更新 → 本家同様の**データ競合**。§9 の unsafe 共有可変で再現。`thread_num=1` (本家デフォルト) では 1 worker が `sweep_orders[0]` を使い決定的。

### 7.13 出力・ロボット制御系
- `pos_to_action(x,y,t_rad) -> Option<usize>`: `ix=floor((x-origin_x)/xy_res)`, `iy=floor((y-origin_y)/xy_res)`, `t=(180*t_rad/PI) as i32`, `it=floor(((t + 360*100) % 360) / t_resolution)`。`final_state` なら `status="goal"` で `None`; `optimal_action` あればそれ; else `None`。
- `make_value_function_map(threshold,x,y,yaw) -> OccupancyGrid`: §10。`it=floor(((yaw/PI*180) as i32 + 360*100) % 360 / t_resolution)`。
- `value_function_writer` / `policy_writer`: §10 のプレーンデータ返却。
- `finished(&mut self) -> (sweep_times: Vec<u32>, deltas: Vec<f64>, finished: bool)`: 全 thread_status を集約。
- `set_cancel` / `end_of_trial` / `arrived` / `set_calculated` / `is_calculated`: status 文字列の本家通りの遷移。

## 8. ValueIteratorLocal — `local.rs`

合成 (`base: ValueIterator`) + 委譲で再現。

- フィールド: `local_ix_min/max`, `local_iy_min/max` (i32), `local_ixy_range` (i32), `local_xy_range` (f64)。
- `set_map_with_occupancy_grid`: `base` の同名を呼び、`local_xy_range=1.0`, `local_ixy_range=(1.0/xy_res) as i32`, `local_ix_min=local_iy_min=0`, `local_ix_max=local_iy_max=local_ixy_range*2`。
- `value_iteration_local(s)` / `action_cost_local(s,a)`: `action_cost` と内容が完全同一 (本家 `actionCostLocal` は `actionCost` と同じ式)。忠実のため別メソッドとして用意するが、計算は base に委譲する。
- `local_value_iteration_loop`: `iix in local_ix_min..=local_ix_max`, `iiy in local_iy_min..=local_iy_max`, `iit in 0..cell_num_t` の順で `value_iteration_local(states[to_index(..)])`。
- `local_value_iteration_worker(id)`: status が canceled/goal の間 `status="executing"` に書き換える先頭ループ (本家どおり) → その後 `while status != canceled/goal { local_value_iteration_loop() }`。背景スレッド前提 (決定的テストは `local_value_iteration_loop` を直接呼ぶ)。
- `set_local_cost(scan, x,y,t)`: レーザ各ビームについて、d=0.1..0.9 の中間点で `inLocalArea` なら `local_penalty /= 2`、ヒット点±2 セルで `inLocalArea` なら `local_penalty = 2048 << PROB_BASE_BIT`。本家の `for(double d=0.1; d<=0.9; d+=0.1)` の f64 刻みを忠実に。
- `set_local_window(x,y)`: ロボット位置中心に local window をクランプ設定。
- `pos_to_action` override: `final_state`→`status="goal"`,`None`; else `optimal_action`。
- `make_local_value_function_map`: §10。

## 9. マルチスレッドと unsafe 設計

- `set_state_transition` (θ並列): 書き込み先がθごとに独立 → 安全。各スレッドが結果を返してメインが格納。**データ競合なし・決定的**。
- `value_iteration_worker` (states 共有並行): 本家は `std::vector<State> states_` を複数スレッドが**ロックなし**で並行 read/write (非 atomic) → 技術的には UB だが x86 で「動く」。**忠実再現**として:
  - `states: Vec<State>` を `value_iteration_worker` 実行中だけ生ポインタ (`*mut State` / `UnsafeCell`) 経由で共有し、`std::thread::scope` 内で `thread_num` スレッドが `unsafe` に同一バッファを更新する。
  - 各 State フィールドは非 atomic (`u64`/`i32`/`bool`)。C++ の非 atomic 競合と同じ語彙。`thread_num=1` では実競合なし→決定的。
  - `unsafe` ブロックには「本家のデータ競合を忠実再現するための意図的 unsafe」である旨のコメントを付す。`status`/`thread_status` はバッチ実行中は不変なので安全側で扱い (break 条件を bool スナップショット化、各スレッドは自分の id 結果を返してメインが格納)、観測値 (sweep_step/delta/finished) に差異が出ないことを確認する。
- 公開 API は単スレッド (`thread_num=1`) を既定の決定的経路として提供し、マルチスレッドは明示的に `thread_num>1` を設定した場合のみ。テストは単スレッド経路で bit 一致を担保する。

## 10. ROS 出力層の代替

`grid_map` / `nav_msgs` への実シリアライズは行わず、**数値内容が同一**のプレーン Rust データを返す。

- `value_function_writer() -> GridLayers`: θ層ごとに `total_cost as f64 / PROB_BASE as f64` を `(cell_num_y, cell_num_x)` で格納したもの (本家 `grid_map` の各レイヤ `to_string(t)` に対応)。
- `policy_writer() -> GridLayers`: θ層ごとに `optimal_action` の `id` (None は `-1.0`) を格納。
- `make_value_function_map(threshold,x,y,yaw) -> OccupancyGrid`: 本家どおり `cost = total_cost/PROB_BASE`; `cost<threshold` なら `(cost/threshold*250) as i32`、`free` なら 250、else 255 を `data` に push。`width=cell_num_x, height=cell_num_y, resolution=xy_res, origin=map_origin*`。
  - ※ `push_back((int)(...))` を `int8` に格納する際の本家挙動 (250/255 が int8 では負値) を**忠実再現**: `250 as u8 as i8` 等のラップを保持。
- `make_local_value_function_map`: 同様 (local window 範囲・origin がロボット相対)。

## 11. 数値オーバーフロー (折り返し) の扱い

- `action_cost` の `cost += (total+penalty+local)*prob` は u64。未到達 free セル (`total_cost=MAX_COST≈2.62e14`) を含むアクションでは `MAX_COST * PROB_BASE ≈ 6.87e19 > u64::MAX(1.84e19)` となり**折り返す**。本家 C++ unsigned 演算の折り返しと一致させるため Rust は `wrapping_add`/`wrapping_mul` を使用。
- この結果、初期スイープでは「未到達 free セルのみを遷移先とするアクション」が折り返しで `MAX_COST` 未満の偽コストを得る、という本家固有の挙動が再現される。ゴール近傍から伝播する正規の有限コストが小さいため min 選択で徐々に washout する。
- `value_iteration` の delta、`value_iteration_worker` の `max_delta >> 18` も本家のシフト・型を忠実に。
- **テストで折り返し値を 1 つ手計算し固定** (回帰防止)。

## 12. テスト戦略

`vi_reference/src/**` の `#[cfg(test)]` + `vi_reference/tests/` 結合テスト。本家バイナリとの実行時突き合わせは環境依存のため**まずアルゴリズム単体の不変条件と手計算値**で固める。

- 定数: `PROB_BASE=262144`, `PROB_BASE_BIT=18`, `MAX_COST=262_144_000_000_000`。
- `cell_delta`: 負座標の `-ix-1` 補正、`it` 非正規化。
- `no_noise_state_transition`: `to_t` の負正規化のみ (>=360 残存)。
- `set_state_transition`: 1 アクション・1θの prob 総和 = 262144; θ絶対 index になっていること (前進/回転アクションで dit がθ依存の絶対値); サブセルサンプル列が 64^3 であること。
- `State::occupancy`: margin penalty の**行跨ぎバグ**を、意図的に `x=0` 近傍・`ix2<0` で隣接行を読む小マップで再現確認。
- `set_state_values`: 距離 + 向き判定 (goal_t_2 の両側判定、t0/t1 切り捨て)。
- `action_cost`: マップ外 / not free で `MAX_COST`; 決定的アクションで `(total+penalty)` 一致; **オーバーフロー折り返し値**の固定。
- `value_iteration` / 単スレッド `value_iteration_worker`: 小さな自由マップ + 単一ゴールで、収束・`optimal_action`・idempotency を照合。
- `ValueIteratorLocal`: `set_local_cost` による `local_penalty` の半減 / 2048<<18 設定、local window のクランプ、`local_value_iteration_loop` の走査範囲。
- マルチスレッド (`thread_num>1`) は十分スイープ後に単スレッドと同一固定点へ収束することを検証。

## 13. 既存コードへの影響

- `vi_rs/Cargo.toml` の `members` に `"vi_reference"` を追加 (1 行)。
- `vi_core` / `vi_algorithm` / `vi_fixtures` / `vi_bench` は**無変更**。
- 既存 `Reference` (16-bit) / FPGA オラクル / 約 20 パリティテストは無傷。
- (任意・将来) `vi_bench` から本家忠実 reference を呼べるようにするのは別タスク。本仕様では行わない。

## 14. 公開 API イメージ

```rust
use vi_reference::{ValueIterator, ValueIteratorLocal, Action, OccupancyGrid, Quaternion};

let actions = vec![
    Action::new("forward", 0.3, 0.0, 0),
    Action::new("back",   -0.2, 0.0, 1),
    Action::new("right",   0.0,-20.0,2),
    Action::new("rightfw", 0.2,-20.0,3),
    Action::new("left",    0.0, 20.0,4),
    Action::new("leftfw",  0.2, 20.0,5),
];
let mut vi = ValueIterator::new(actions, /*thread_num=*/1);
vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
vi.set_goal(gx, gy, gt);
vi.run_value_iteration(/*times=*/2000); // 単スレッド・決定的
let vf = vi.value_function_writer();
let pol = vi.policy_writer();
```

---

## 非目標 (再掲)
- 本家アルゴリズムの最適化・バグ修正・並列化改善。
- ROS / grid_map ランタイム再現。
- 既存 vi_core/vi_algorithm 契約の変更。
