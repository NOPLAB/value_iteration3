# 本家(ROS1) vs vi_ros2(ROS2) 比較ベンチマーク 設計仕様

- 起票日: 2026-06-04
- 関連: [vi_ros2 設計](2026-05-29-vi-ros2-design.md)、[vi_rs algorithm port](2026-05-22-vi-rs-algorithm-port-design.md)、`../value_iteration` (本家 ROS1 catkin パッケージ)

## 1. 目的とスコープ

本家 `../value_iteration` (ROS1 catkin/roscpp) と、こちらの `vi_ros2/vi_node` (ROS2 Humble + rclrs) を、
**どちらも実 ROS ノードとして Docker 上で起動し、action goal で end-to-end に駆動**して、
同一マップ・同一ゴールに対する **速度** と **結果一致度** を比較する。

**含む:**
- 本家を ROS1 Noetic コンテナで headless 起動 (map_server + vi_node、RViz/Gazebo/localization なし)
- こちらを ROS2 Humble コンテナで vi_node 実起動 (`Vi` アクション駆動)
- 両者を action goal で駆動し、収束までのウォールクロック時間と sweep 回数を計測
- 価値関数 (ステップ単位) と 方策 (action id) を `(H, W, N_THETA)` 配列として抽出し一致度を算出
- 2 コンテナを逐次実行するオーケストレータと比較レポート生成

**含まない:**
- ROS1 ⇄ ROS2 ブリッジ (両者は通信せず、各自が計算して共有ボリュームに結果を dump)
- online 走行 (`cmd_vel` / tf / LaserScan)。両者とも `online=false` の純プランニング
- 本家アルゴリズムの改変 (外部リポジトリは read-only。マウントしてビルドのみ)
- Ultra96 / FPGA 実機

**前提となる確認済み事項 (調査結果):**
- アクション集合が**完全一致**: 本家 `launch/navigation_house.launch` の 6 アクション
  `forward(0.3,0) / back(-0.2,0) / right(0,-20) / rightfw(0.2,-20) / left(0,20) / leftfw(0.2,20)`
  が `vi_core` の `ACTION_FW`/`ACTION_ROT` と値・**ID 順まで一致**。→ 方策(action id)比較が意味を持つ。
- `theta_cell_num = 60 = N_THETA` 一致。
- 価値の単位が**同一**: 本家は各 state に `penalty_ = prob_base_` のベースライン (= 1 ステップ分) を持ち、
  報告値 = `total_cost_ / prob_base_` ≒ ゴールまでのステップ数 + ペナルティ。
  こちらは `cost_of = neighbor_value + penalty + STEP_COST(=1)`。両者とも「ステップ数 + ペナルティ」単位。
  → 価値の直接比較 (絶対 RMSE) が可能。

## 2. 全体アーキテクチャ

```
vi_compare/
├─ docker/
│   ├─ Dockerfile.ros1        # ros:noetic-robot + grid_map + map_server; ../value_iteration を catkin_make
│   └─ docker-compose.yml     # 2 サービス(noetic / humble) + 共有 results ボリューム
├─ ros1/
│   ├─ bench.launch           # map_server(house.yaml) + vi_node(online:=false) のみ
│   └─ bench_client.py        # rospy: goal送信→delta監視→収束計時&preempt→/value,/policy取得→npy保存
├─ ros2/
│   └─ bench_client.py        # rclpy: Viアクションgoal送信→result待ち→計時 (value/policy は vi_node が dump)
├─ compare/
│   └─ compare.py             # 両 npy を整列・正規化し RMSE/相関/方策一致率/速度表→report.md
├─ params.yaml                # 共通ベンチパラメータ (goal, safety_radius_penalty, threshold, thread_num ...)
└─ results/                   # value_{ros1,ros2}.npy, policy_{ros1,ros2}.npy, timing_{ros1,ros2}.json, report.md
```

オーケストレータ (`scripts/compare_bench.sh`、`make compare-bench`) は **逐次** に実行する:

1. ROS1 コンテナ: `bench.launch` 起動 → `bench_client.py` が goal 駆動 → `value_ros1.npy`/`policy_ros1.npy`/`timing_ros1.json` を `results/` に dump。
2. ROS2 コンテナ: `vi_node` 起動 (`bench_dump_path=results/...`) → `bench_client.py` が `Vi` goal 駆動 → vi_node が `value_ros2.npy`/`policy_ros2.npy` を dump、client が `timing_ros2.json` を書く。
3. `compare.py` (どちらかのコンテナ or ホスト Python) が両者を読み `report.md` 生成。

逐次実行は **CPU 競合を避けて計時を公平にする** ため。並列化しない。

## 3. 本家 (ROS1) 側

### 3.1 イメージ (`docker/Dockerfile.ros1`)
- ベース: `ros:noetic-robot`
- 追加 apt: `ros-noetic-grid-map`(メタ; grid_map_ros/msgs 一式), `ros-noetic-map-server`, `python3-numpy`
- `../value_iteration` を catkin ワークスペース `src/value_iteration` にマウント (read-only マウント + ビルドは別ボリューム) し `catkin_make`。
- ベンチ用スクリプト (`ros1/bench.launch`, `ros1/bench_client.py`) は `vi_compare/ros1/` をマウントして使用 (本家リポジトリは改変しない)。

### 3.2 headless launch (`ros1/bench.launch`)
本家の `navigation_house.launch` から、計算に不要な要素 (turtlebot3 / Gazebo / emcl / RViz / static_transform / online) を除去し、以下のみ:
- `map_server` (`house.yaml`) → `/static_map` サービス + `/map` トピック
- `vi_node`:
  - `online: false`
  - `theta_cell_num: 60`
  - `action_list`: 上記 6 アクション (本家の rosparam dict 記法)
  - `safety_radius`, `safety_radius_penalty`, `goal_margin_radius`, `goal_margin_theta`, `map_type: occupancy` を `params.yaml` 由来の整合値で設定
  - `thread_num`: `params.yaml` 由来 (既定 1)

### 3.3 bench client (`ros1/bench_client.py`, rospy)
本家は **offline では自動収束停止しない** (`ValueIterator::valueIterationWorker` の delta 閾値 break はコメントアウト、worker は `INT_MAX` sweep)。
このためクライアントが収束を検知して止める:

1. `/vi_controller` (`ViAction`) に goal (`PoseStamped`; `params.yaml` の x,y,yaw) を送信。
2. feedback (`deltas` = 各スレッド最新 sweep の最大変化, `current_sweep_times`) を購読。
3. **全スレッドの max delta ≤ `delta_threshold`** を満たした最初の時刻を「収束時刻」として記録 (goal accept 時刻からの経過)。同時に sweep 数を記録。
4. goal を `cancel_goal()` で preempt (worker 停止)。
5. `/value`・`/policy` サービス (`grid_map_msgs/GetGridMap`) を呼び、60 層 (theta=0..59) を取得。
   - 各層 float 値: value = `total_cost_/prob_base_`、policy = optimal action id (未定 = -1)。
   - `(H, W, 60)` の numpy 配列に整形して `value_ros1.npy` / `policy_ros1.npy` 保存。
   - **grid_map の軸順は OccupancyGrid と異なり得る**ため、`outer_start_index`/`inner_start_index`/`data layout` を厳密にデコードしたうえで、
     既知の障害物セル集合 (house.pgm から導出) と value=unreachable セルの突き合わせで転置/反転を検証・自動補正する。
6. `timing_ros1.json` (`elapsed_sec`, `sweeps`, `converged`, `thread_num`, `delta_threshold`) を保存。

## 4. こちら (ROS2) 側 — vi_node 最小改修込み

### 4.1 イメージ
既存 `vi_ros2/docker/Dockerfile` (ros:humble + ros2_rust + rclrs) を流用。

### 4.2 vi_node 改修 (本番挙動を変えない範囲で)
現状 `value_function` トピックは theta=0 スライスのみ、`policy` は -1 スタブで**結果比較に使えない**ため、専用 dump を追加する:

1. **パラメータ `bench_dump_path` (string, 既定 "")**: 空なら無効 = 既存挙動不変。
2. **`sweep_thread` worker が終了時 (収束 or キャンセル) に最終 `value`(Array3) と `policy`(Array3) を共有スロットに格納**:
   - policy は `vi_algorithm::optimal_action_at` を全セルに適用して構築。
   - 共有スロット: `Arc<Mutex<Option<(Array3<Value>, Array3<ActionIdx>)>>>` を `SweepHandle` に追加。worker が return 直前に格納。
   - dump 無効時は policy 全走査コストを払わないようフラグでガード。
3. **action コールバック**が worker join 後、`bench_dump_path` 指定時のみ `value_ros2.npy` / `policy_ros2.npy` を書き出す。

### 4.3 bench client (`ros2/bench_client.py`, rclpy)
1. `vi_controller` (`Vi` アクション) に goal (`PoseStamped`; `params.yaml`) を送信。
2. result (`finished`) を待ち、goal 送信〜result のウォールクロックを計測。
3. `timing_ros2.json` (`elapsed_sec`, `sweeps`(feedback 最終値), `converged`, `thread_num`, `delta_threshold`) を保存。
4. value/policy は vi_node が dump 済み。

収束判定はこちら側で完結 (vi_node worker は `stats.converged` で自動停止)。
`delta_threshold` は vi_node の `delta_threshold` パラメータに渡し、本家と同一基準にする。

## 5. 公平性・パラメータ整合 (`params.yaml`)

| 項目 | 値 / 方針 |
|---|---|
| マップ | `../value_iteration/maps/house.pgm` + `house.yaml` (0.05 m/pix, origin [-10,-10]) |
| ゴール | 固定 1 点 (x, y, yaw) を `params.yaml` で指定 (両者同一) |
| アクション | 6 種 (一致済み)、ID 順一致 |
| theta_cell_num | 60 |
| safety_radius | 0.2 m |
| safety_radius_penalty | **両者で有効な値に統一 (既定 30)**。本家 launch の 100000 は u16 で溢れるため不使用 |
| goal_margin_radius | 0.3 m |
| goal_margin_theta | 15 deg |
| 収束判定 | **1 sweep で delta == 0 (完全収束)** を既定。`delta_threshold` で緩和可、`max_sweeps`/`timeout_sec` の安全弁あり |
| thread_num | 既定 **1 (純アルゴリズム比較)**。多スレッドも可だが本家=sweep_order 並列 / こちら=rayon で戦略が別物である旨レポートに明記 |

ペナルティ意味論の対応: 本家 `penalty_ = margin_penalty*prob_base + prob_base` (near-obstacle) / `prob_base` (free)。
報告値換算で free=1、near-obstacle = `safety_radius_penalty + 1`。
こちらは free=`STEP_COST`(1)、near-obstacle = `safety_radius_penalty + STEP_COST`。`safety_radius_penalty` を一致させれば対応する。
※ 本家は確率的遷移 (sub-cell サンプリング) の重み付き平均、こちらは `vi_fixtures` の `PaperMonteCarlo` (本家準拠) を使用。完全ビット一致ではなく**近似一致**で評価する。

## 6. 比較指標と出力 (`compare/compare.py`)

両 npy を読み、**両者で可達 (value < unreachable sentinel) なセル**に限定して算出:

- **価値一致度**: RMSE / MAE / 最大絶対差 / Pearson 相関 / Spearman 順位相関 (単位差・外れ値に頑健)
- **方策一致度**: action id 一致率 (全 theta、および theta=0 スライス別)。可達セルのみ対象
- **速度**: ウォールクロック (秒)、sweep 回数、収束フラグ、thread_num を表に
- **出力**: `results/report.md` (Markdown 表 + サマリ)、任意で theta=0 価値ヒートマップ画像 (`matplotlib`、依存追加可否は実装時判断)

unreachable / obstacle セルの定義整合:
- 本家 value=`max_cost_/prob_base_`(=1e9) を unreachable とみなす閾値処理。
- こちら value=`MAX_VALUE`(0xFFFF) を unreachable。
- 障害物セルは house.pgm から判定し、両者の unreachable 集合の一致もレポートに含める (整列検証を兼ねる)。

## 7. 実行方法

```sh
make compare-bench        # ビルド→ROS1計算→ROS2計算→比較レポート (results/report.md)
```

個別ターゲット (デバッグ用) も用意:
- `make compare-build` (両イメージ build)
- `make compare-ros1` / `make compare-ros2` (片側のみ計算)
- `make compare-report` (既存 npy から report 再生成)

## 8. リスクと段階計画

### 最大リスク: vi_node のビルド未検証
`vi_ros2/vi_node/src/main.rs` は rclrs API を推測実装 (`TODO(Task 11)` 多数)、`make ros2-build` の疎通が現状未確認。
end-to-end 構成を選択したため、**Phase 0 (vi_node を Humble イメージでビルド & 起動可能にする) が前提作業**であり、ここが想定外に大きくなる可能性がある。

**フォールバック (Phase 0 が過大だった場合の縮退案、本スコープ外だが記録):**
vi_node の代わりに、vi_node が link する `vi_rs` を直接叩く headless Rust バイナリで ROS2 側を代替する案。
ただし「実ノード end-to-end」要件からは外れるため、採用時はユーザ再確認が必要。

### 段階計画
- **Phase 0**: vi_node ビルド疎通 (`make ros2-build`) + rclrs API 修正 (action server / pub / sub / param / timer)
- **Phase 1**: ROS1 Noetic イメージ + `bench.launch` + `bench_client.py` (rospy)。grid_map 軸デコード確立
- **Phase 2**: vi_node に dump 改修 (`bench_dump_path` + 共有スロット) + `bench_client.py` (rclpy)
- **Phase 3**: `compare.py` + `docker-compose.yml` + `scripts/compare_bench.sh` + Makefile ターゲット
- **Phase 4**: 実走・パラメータ調整・`report.md` 確認

## 9. 受け入れ基準

- `make compare-bench` 一発で両コンテナが起動し、`results/report.md` が生成される。
- `report.md` に (1) 速度表 (ROS1/ROS2 の elapsed・sweep)、(2) 価値一致度 (RMSE/相関)、(3) 方策一致率 が含まれる。
- 同一マップ・ゴール・パラメータで再実行したとき、結果指標が再現する (計時はハード次第でぶれる旨を注記)。
- 本家リポジトリ (`../value_iteration`) に変更を加えない。
- vi_node の本番挙動 (`bench_dump_path` 未設定時) が改修前と不変。
