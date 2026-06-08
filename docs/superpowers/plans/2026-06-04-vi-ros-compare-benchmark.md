# 本家(ROS1) vs vi_ros2(ROS2) 比較ベンチマーク Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 本家 `../value_iteration` (ROS1) と `vi_ros2/vi_node` (ROS2) を Docker 上で実ノードとして起動し、`house.pgm`・同一ゴールで速度(ウォールクロック/sweep数)と結果(価値RMSE・相関/方策一致率)を比較するベンチマークを作る。

**Architecture:** ROS1 Noetic コンテナと ROS2 Humble コンテナを **逐次** 起動し、各々が action goal で駆動された VI を計算して `(H,W,N_THETA)` の value/policy を `vi_compare/results/` に npy で dump。`compare/compare.py` が両者を整列(障害物マスクで自動補正)・正規化して `report.md` を生成する。本家リポジトリは無改変、vi_node には `bench_dump_path` 指定時のみ働く dump 改修を入れる。

**Tech Stack:** ROS1 Noetic (roscpp/rospy/grid_map/map_server), ROS2 Humble (rclrs/rclpy/ros2_rust), Rust (vi_node/vi_rs, 依存追加なしの手書き npy writer), Python3+numpy, Docker / docker compose, Make。

**確定済みの前提 (調査結果):**
- マップ: `../value_iteration/maps/house.pgm` = **384×384**, 0.05 m/pix, origin [-10,-10], `house.yaml`。
- アクション 6 種が本家 `navigation_house.launch` と `vi_core::{ACTION_FW,ACTION_ROT}` で **値・ID順とも一致**: `forward(0.3,0)/back(-0.2,0)/right(0,-20)/rightfw(0.2,-20)/left(0,20)/leftfw(0.2,20)`。
- `theta_cell_num=60=N_THETA`。価値は両者「ステップ数+ペナルティ」同単位。
- 本家は offline で**自動収束停止しない**(`valueIterationWorker` は `INT_MAX` sweep) → client が delta 監視して preempt する。
- 本家の value/policy 全解像度出力は `/value`・`/policy` (grid_map_msgs/GetGridMap) のみ (theta 60 層)。
- vi_node 現状: `value_function` は theta=0 のみ・`policy` は -1 スタブ → 結果比較に使えないため dump 改修が必要。
- `vi_algorithm::optimal_action_at(ctx, ix, iy, it) -> u8` は obstacle/goal/blocked で 0 を返す → dump 側で value==MAX_VALUE / obstacle / goal を -1 にマスクする。
- `vi_interfaces` は `rosidl_default_generators` 使用 → Python 型支援も生成され rclpy クライアント可。

**spec:** `docs/superpowers/specs/2026-06-04-vi-ros-compare-benchmark-design.md`

---

## File Structure

```
vi_compare/
├─ docker/
│   ├─ Dockerfile.ros1            # Create: ROS1 Noetic + grid_map + map_server
│   └─ docker-compose.yml         # Create: ros1 / ros2 サービス + results ボリューム
├─ params.yaml                    # Create: goal + client 閾値 + 計画パラメータ(正典)
├─ ros1/
│   ├─ bench.launch               # Create: map_server + vi_node(online:=false)
│   ├─ bench_client.py            # Create: rospy goal駆動→収束計時→/value,/policy→npy
│   └─ run_ros1_bench.sh          # Create: catkin_make→roslaunch→client→shutdown
├─ ros2/
│   ├─ vi_node_params.yaml        # Create: ROS2 vi_node パラメータ(params.yaml をミラー)
│   ├─ bench_client.py            # Create: rclpy Viアクション goal駆動→result計時→timing.json
│   └─ run_ros2_bench.sh          # Create: build→vi_node起動→client→shutdown
├─ compare/
│   ├─ compare.py                 # Create: 整列+指標+report.md
│   └─ test_compare.py            # Create: compare.py コア関数の自己テスト
└─ results/.gitkeep               # Create: 出力先

scripts/compare_bench.sh          # Create: 全体オーケストレータ
Makefile                          # Modify: compare-* ターゲット追加 (148行目付近)
vi_ros2/vi_node/src/npy.rs        # Create: 依存なし npy writer
vi_ros2/vi_node/src/lib.rs        # Modify: `pub mod npy;`
vi_ros2/vi_node/src/sweep_thread.rs # Modify: DumpData + compute_policy + spawn_sweep に dump_slot
vi_ros2/vi_node/src/main.rs       # Modify: bench_dump_path param + dump slot + npy 書き出し
```

---

## Phase 0 — vi_node を Humble でビルド & 起動可能にする (前提スパイク)

> **性質:** `main.rs` の rclrs API は推測実装 (`TODO(Task 11)` 多数) で、実コード修正は**コンパイラ出力駆動**になる。事前に正確な diff は書けないため、本フェーズは「探索 + 修正ループ」として exact コマンドと完了条件(DoD)で定義する。真実のソースはイメージ内 `/ros2_rust_ws/src/ros2_rust/rclrs/` にある。

### Task 0.1: Humble イメージをビルドし、現状のビルドエラーを採取

**Files:** (なし。調査)

- [ ] **Step 1: イメージビルド**

Run: `make ros2-docker`
Expected: `vi_ros2_dev:humble` が作成される (ros2_rust の rclrs まで colcon build 済み)。失敗時は Dockerfile のネットワーク/apt を確認。

- [ ] **Step 2: 現状の colcon ビルドを実行しエラーをログ化**

Run: `make ros2-build 2>&1 | tee /tmp/ros2_build_phase0.log`
Expected: 失敗が想定される。`/tmp/ros2_build_phase0.log` に rclrs API 不一致のコンパイルエラーが並ぶ。

- [ ] **Step 3: 失敗箇所を API サーフェス別に分類**

エラーを以下の観点で `vi_node/src/main.rs` のどの呼び出しが該当するか一覧化 (コメントメモで可):
- Context/Executor/Node 生成 (`Context::default_from_env`, `create_basic_executor`, `create_node`)
- パラメータ宣言 (`declare_parameter::<T>().default().mandatory().get()`、`Vec<String>`/`Vec<f64>` 型)
- Subscription (`create_subscription`, `"map".transient_local().reliable().keep_last(1)`)
- Publisher / wall timer (`create_publisher`, `create_wall_timer`, `clock().now().to_msg()`, publish の by-ref/by-value)
- Action server (`create_action_server`, `RequestedGoal`/`accept`/`execute`/`feedback_publisher`/`succeeded_with`/`aborted_with`、型パス `vi_interfaces::action::Vi{,_Goal,_Result,_Feedback}`)
- tokio 利用可否 (`spawn_blocking`, `time::interval`)

### Task 0.2: rclrs API を実ソースに合わせて修正し、ビルドを通す

**Files:**
- Modify: `vi_ros2/vi_node/src/main.rs` (コンパイラ出力に応じた箇所)
- Modify (必要時): `vi_ros2/vi_node/Cargo.toml` (rclrs/std_msgs/nav_msgs/geometry_msgs/tokio の version を colcon 解決値に合わせる)

- [ ] **Step 1: 実 API をソースで確認**

Run (コンテナ内シェル): `make ros2-shell` → 
`grep -rn "pub fn create_action_server\|pub fn create_subscription\|pub fn create_publisher\|pub fn declare_parameter\|pub fn create_wall_timer" /ros2_rust_ws/src/ros2_rust/rclrs/src`
さらに rclrs の examples: `ls /ros2_rust_ws/src/ros2_rust/examples` を参照し、action server / pub-sub / params の実呼び出し例を確認。

- [ ] **Step 2: main.rs を実 API に合わせて修正**

分類した各サーフェスについて、`/tmp/ros2_build_phase0.log` のエラーと実ソースのシグネチャを突き合わせ、`TODO(Task 11)` 周辺を修正する。型パスは `grep -rn "Vi_Goal\|mod action" install/` で生成物を確認。

- [ ] **Step 3: ビルドを反復実行し緑にする**

Run: `make ros2-build`
Expected: 最終的に exit 0 (`colcon build` 成功)。エラーが残る間は Step 1-2 を反復。

- [ ] **Step 4: 既存ユニットテストの非ROS部分が壊れていないこと**

Run: `cd vi_ros2/vi_node && cargo test --lib --no-default-features`
Expected: bridge/solver_factory/sweep_thread のユニットテストが PASS (ROS 非依存部分)。

- [ ] **Step 5: Commit**

```bash
git add vi_ros2/vi_node/src/main.rs vi_ros2/vi_node/Cargo.toml
git commit -m "fix(vi_node): make rclrs API compile in Humble image (Task 11)"
```

### Task 0.3: vi_node の end-to-end スモーク (合成マップ + 1 goal)

**Files:**
- Create: `vi_compare/ros2/smoke.sh` (一時、後で run スクリプトに発展)

- [ ] **Step 1: スモークスクリプト**

```bash
#!/usr/bin/env bash
# vi_compare/ros2/smoke.sh — tiny synthetic /map + one Vi goal
set -e
source /opt/ros/humble/setup.sh
source /ros2_rust_ws/install/local_setup.sh
source /workspace/vi_ros2_ws/install/local_setup.sh
# 4x4 free OccupancyGrid on /map (transient_local)
ros2 topic pub --once -q /map nav_msgs/msg/OccupancyGrid \
  '{info: {width: 4, height: 4, resolution: 0.05, origin: {position: {x: -0.1, y: -0.1}}}, data: [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]}' &
ros2 run vi_node vi_node --ros-args -p solver:=reference -p map_wait_sec:=15 &
NODE=$!
sleep 5
# send a goal at cell center; expect result.finished
timeout 60 ros2 action send_goal /vi_controller vi_interfaces/action/Vi \
  '{goal: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}'
kill $NODE 2>/dev/null || true
```

- [ ] **Step 2: スモーク実行**

Run (コンテナ内): `make ros2-build && make ros2-shell` 後に `bash vi_compare/ros2/smoke.sh`
Expected: action が `Result: finished=true` (または収束して succeeded) を返す。ノードが panic せず goal を処理できることを確認。

- [ ] **Step 3: Commit**

```bash
git add vi_compare/ros2/smoke.sh
git commit -m "test(vi_node): add end-to-end smoke for action server"
```

**Phase 0 DoD:** `make ros2-build` が緑、`vi_node` が `/map` を受けて `Vi` goal を処理し result を返す。

---

## Phase 1 — ROS1 Noetic 側 (headless 起動 + rospy client)

### Task 1.1: ROS1 イメージ

**Files:**
- Create: `vi_compare/docker/Dockerfile.ros1`

- [ ] **Step 1: Dockerfile を作成**

```dockerfile
FROM ros:noetic-robot
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-grid-map \
      ros-noetic-map-server \
      python3-numpy \
      python3-yaml \
      build-essential \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /catkin_ws
CMD ["/bin/bash"]
```

- [ ] **Step 2: イメージビルド確認**

Run: `docker build -t vi_compare_ros1:noetic vi_compare/docker -f vi_compare/docker/Dockerfile.ros1`
Expected: ビルド成功。`ros-noetic-grid-map` が grid_map_msgs/grid_map_ros を含む。

- [ ] **Step 3: Commit**

```bash
git add vi_compare/docker/Dockerfile.ros1
git commit -m "build(vi_compare): add ROS1 Noetic image with grid_map + map_server"
```

### Task 1.2: 共通パラメータ `params.yaml`

**Files:**
- Create: `vi_compare/params.yaml`

- [ ] **Step 1: params.yaml を作成**

```yaml
# 比較ベンチ共通パラメータ (正典)。
# vi_node の計画パラメータは ros1/bench.launch と ros2/vi_node_params.yaml に "同期して" 反映する。
# goal と client 閾値はここだけが真実 (両 client が読む)。
goal:
  x: 3.0          # m (house.yaml origin=[-10,-10], 0.05 m/pix; 後で free セルへ Phase 4 で確定)
  y: 0.0          # m
  yaw_deg: 0.0    # deg
client:
  delta_threshold: 0     # 1 sweep の最大変化がこれ以下で収束
  max_sweeps: 3000       # 安全弁
  timeout_sec: 1200      # 安全弁
planning:               # ↓ bench.launch / vi_node_params.yaml にミラー
  theta_cell_num: 60
  safety_radius: 0.2
  safety_radius_penalty: 30
  goal_margin_radius: 0.3
  goal_margin_theta: 15
  thread_num: 1
```

- [ ] **Step 2: Commit**

```bash
git add vi_compare/params.yaml
git commit -m "feat(vi_compare): add canonical benchmark params.yaml"
```

### Task 1.3: ROS1 headless launch

**Files:**
- Create: `vi_compare/ros1/bench.launch`

- [ ] **Step 1: bench.launch を作成 (RViz/Gazebo/localization なし、online:=false)**

```xml
<launch>
  <arg name="map_yaml" default="/src_value_iteration/maps/house.yaml"/>

  <!-- planning params: keep in sync with vi_compare/params.yaml :: planning -->
  <rosparam>
    vi_node:
      action_list:
        - {name: forward, onestep_forward_m: 0.3,  onestep_rotation_deg: 0.0}
        - {name: back,    onestep_forward_m: -0.2, onestep_rotation_deg: 0.0}
        - {name: right,   onestep_forward_m: 0.0,  onestep_rotation_deg: -20.0}
        - {name: rightfw, onestep_forward_m: 0.2,  onestep_rotation_deg: -20.0}
        - {name: left,    onestep_forward_m: 0.0,  onestep_rotation_deg: 20.0}
        - {name: leftfw,  onestep_forward_m: 0.2,  onestep_rotation_deg: 20.0}
  </rosparam>

  <node pkg="map_server" name="map_server" type="map_server" args="$(arg map_yaml)"/>

  <node pkg="value_iteration" name="vi_node" type="vi_node" output="screen" required="true">
    <param name="online" value="false"/>
    <param name="theta_cell_num" value="60"/>
    <param name="thread_num" value="1"/>
    <param name="safety_radius" value="0.2"/>
    <param name="safety_radius_penalty" value="30.0"/>
    <param name="goal_margin_radius" value="0.3"/>
    <param name="goal_margin_theta" value="15"/>
    <param name="map_type" value="occupancy"/>
  </node>
</launch>
```

- [ ] **Step 2: Commit**

```bash
git add vi_compare/ros1/bench.launch
git commit -m "feat(vi_compare): add ROS1 headless bench launch"
```

### Task 1.4: rospy bench client

**Files:**
- Create: `vi_compare/ros1/bench_client.py`

- [ ] **Step 1: bench_client.py を作成**

```python
#!/usr/bin/env python3
"""ROS1 bench client: send goal, time convergence via feedback deltas,
preempt at convergence, fetch /value & /policy gridmaps, dump npy + timing."""
import sys, json, time, os
import numpy as np
import rospy, actionlib, yaml
from value_iteration.msg import ViAction, ViGoal
from grid_map_msgs.srv import GetGridMap
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

UNREACHABLE = 1e6  # value >= this => unreachable (max_cost_/prob_base_ == 1e9)

class Bench:
    def __init__(self, p):
        self.thr = p['client']['delta_threshold']
        self.max_sweeps = p['client']['max_sweeps']
        self.timeout = p['client']['timeout_sec']
        self.t0 = None
        self.converged = False
        self.elapsed = None
        self.sweeps = None
        self.client = actionlib.SimpleActionClient('/vi_controller', ViAction)

    def feedback_cb(self, fb):
        deltas = list(fb.deltas.data)
        sweeps = list(fb.current_sweep_times.data)
        if not deltas:
            return
        mx = max(deltas)
        cur = max(sweeps) if sweeps else 0
        rospy.loginfo("sweep=%d max_delta=%g", cur, mx)
        if self.converged or self.elapsed is not None:
            return
        if cur >= 1 and (mx <= self.thr or cur >= self.max_sweeps):
            self.elapsed = time.monotonic() - self.t0
            self.sweeps = int(cur)
            self.converged = bool(mx <= self.thr)
            self.client.cancel_goal()

    def run(self, gx, gy, gyaw_deg):
        rospy.loginfo("waiting for action server /vi_controller ...")
        self.client.wait_for_server()
        goal = ViGoal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        q = quaternion_from_euler(0.0, 0.0, np.deg2rad(gyaw_deg))
        ps.pose.orientation.x, ps.pose.orientation.y = q[0], q[1]
        ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
        goal.goal = ps
        self.t0 = time.monotonic()
        self.client.send_goal(goal, feedback_cb=self.feedback_cb)
        deadline = self.t0 + self.timeout
        while not rospy.is_shutdown() and self.elapsed is None and time.monotonic() < deadline:
            rospy.sleep(0.02)
        if self.elapsed is None:  # hard timeout
            self.elapsed = time.monotonic() - self.t0
            self.sweeps = -1
            self.converged = False
            self.client.cancel_goal()
        self.client.wait_for_result(rospy.Duration(15))

def decode_gridmap(gm):
    """Return ndarray [theta, ix, iy] of the grid_map layers '0'..'N-1'."""
    res = gm.info.resolution
    rows = int(round(gm.info.length_x / res))   # x dimension (== cell_num_x)
    cols = int(round(gm.info.length_y / res))   # y dimension (== cell_num_y)
    idx = {name: i for i, name in enumerate(gm.layers)}
    n_theta = len(gm.layers)
    out = np.full((n_theta, rows, cols), np.nan, dtype=np.float64)
    for t in range(n_theta):
        arr = np.array(gm.data[idx[str(t)]].data, dtype=np.float64)
        # grid_map: Eigen column-major; map.at(Index(ix,iy)) => M[ix, iy]
        out[t] = arr.reshape((rows, cols), order='F')
    return out  # [theta, ix, iy]

def fetch(name):
    rospy.loginfo("waiting for service %s ...", name)
    rospy.wait_for_service(name, timeout=60)
    return rospy.ServiceProxy(name, GetGridMap)().map

def main():
    params_path, out_dir = sys.argv[1], sys.argv[2]
    with open(params_path) as f:
        p = yaml.safe_load(f)
    rospy.init_node('vi_bench_client_ros1')
    b = Bench(p)
    g = p['goal']
    b.run(g['x'], g['y'], g['yaw_deg'])

    value = decode_gridmap(fetch('/value'))    # [theta, ix, iy], step units
    policy = decode_gridmap(fetch('/policy'))  # [theta, ix, iy], action id or -1
    # canonical [iy, ix, theta] to match vi_ros2 (H=y, W=x, theta)
    value = np.transpose(value, (2, 1, 0))
    policy = np.transpose(policy, (2, 1, 0))

    os.makedirs(out_dir, exist_ok=True)
    np.save(os.path.join(out_dir, 'value_ros1.npy'), value.astype(np.float64))
    np.save(os.path.join(out_dir, 'policy_ros1.npy'), policy.astype(np.float64))
    timing = dict(elapsed_sec=b.elapsed, sweeps=b.sweeps, converged=b.converged,
                  thread_num=p['planning']['thread_num'],
                  delta_threshold=b.thr, side='ros1')
    with open(os.path.join(out_dir, 'timing_ros1.json'), 'w') as f:
        json.dump(timing, f, indent=2)
    rospy.loginfo("ROS1 bench done: %s", timing)

if __name__ == '__main__':
    main()
```

- [ ] **Step 2: 構文チェック (依存なしで import 部以外を検証)**

Run: `python3 -m py_compile vi_compare/ros1/bench_client.py`
Expected: 構文エラーなし (ROS import の実体は実行時にコンテナで解決)。

- [ ] **Step 3: Commit**

```bash
git add vi_compare/ros1/bench_client.py
git commit -m "feat(vi_compare): add ROS1 rospy bench client (goal + gridmap dump)"
```

### Task 1.5: ROS1 実行スクリプト + 単体実行

**Files:**
- Create: `vi_compare/ros1/run_ros1_bench.sh`

- [ ] **Step 1: run_ros1_bench.sh を作成**

```bash
#!/usr/bin/env bash
# Build value_iteration, launch headless, run bench client, shutdown.
# Expects mounts: /src_value_iteration (本家, ro), /workspace (new repo), /results
set -e
source /opt/ros/noetic/setup.bash
mkdir -p /catkin_ws/src
ln -sfn /src_value_iteration /catkin_ws/src/value_iteration
cd /catkin_ws
if [ ! -f devel/setup.bash ]; then
  catkin_make
fi
source devel/setup.bash
roslaunch /workspace/vi_compare/ros1/bench.launch \
  map_yaml:=/src_value_iteration/maps/house.yaml &
LAUNCH_PID=$!
trap 'kill $LAUNCH_PID 2>/dev/null || true' EXIT
python3 /workspace/vi_compare/ros1/bench_client.py \
  /workspace/vi_compare/params.yaml /results
```

- [ ] **Step 2: ROS1 側を単体実行して npy を生成**

Run:
```bash
docker run --rm \
  -v /home/nop/dev/mywork/value_iteration:/src_value_iteration:ro \
  -v $(pwd):/workspace \
  -v $(pwd)/vi_compare/results:/results \
  vi_compare_ros1:noetic \
  bash /workspace/vi_compare/ros1/run_ros1_bench.sh
```
Expected: `vi_compare/results/value_ros1.npy` (shape (384,384,60)), `policy_ros1.npy`, `timing_ros1.json` が生成。`timing_ros1.json` の `converged: true`、`elapsed_sec`/`sweeps` が記録される。

- [ ] **Step 3: npy の健全性確認**

Run:
```bash
python3 -c "import numpy as np; v=np.load('vi_compare/results/value_ros1.npy'); print(v.shape, v.dtype, (v<1e6).sum(), 'reachable')"
```
Expected: `(384, 384, 60) float64` と、可達セル数が 0 でない (= ゴール到達計算が走った)。

- [ ] **Step 4: Commit**

```bash
git add vi_compare/ros1/run_ros1_bench.sh
git commit -m "feat(vi_compare): add ROS1 runner script + verified npy dump"
```

---

## Phase 2 — vi_node dump 改修 + rclpy client

### Task 2.1: 依存なし npy writer (Rust, TDD)

**Files:**
- Create: `vi_ros2/vi_node/src/npy.rs`
- Modify: `vi_ros2/vi_node/src/lib.rs`

- [ ] **Step 1: 失敗するテストを書く**

`vi_ros2/vi_node/src/npy.rs`:

```rust
//! Minimal dependency-free `.npy` writer for Array3<u16>/<i16> (C order).

use std::fs::File;
use std::io::{self, Write};
use ndarray::Array3;

fn write_header(f: &mut File, descr: &str, shape: &[usize]) -> io::Result<()> {
    let shape_str = shape
        .iter()
        .map(|d| d.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    let dict = format!(
        "{{'descr': '{}', 'fortran_order': False, 'shape': ({},), }}",
        descr, shape_str
    );
    let prefix = 10usize; // magic(6) + version(2) + header_len(2)
    let mut header = dict;
    let unpadded = prefix + header.len() + 1; // +1 for trailing '\n'
    let pad = (64 - (unpadded % 64)) % 64;
    for _ in 0..pad {
        header.push(' ');
    }
    header.push('\n');
    let hlen = header.len() as u16;
    f.write_all(b"\x93NUMPY")?;
    f.write_all(&[0x01, 0x00])?;
    f.write_all(&hlen.to_le_bytes())?;
    f.write_all(header.as_bytes())?;
    Ok(())
}

pub fn write_u16(path: &str, arr: &Array3<u16>) -> io::Result<()> {
    let std = arr.as_standard_layout();
    let mut f = File::create(path)?;
    write_header(&mut f, "<u2", std.shape())?;
    let mut bytes = Vec::with_capacity(std.len() * 2);
    for &v in std.iter() {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    f.write_all(&bytes)
}

pub fn write_i16(path: &str, arr: &Array3<i16>) -> io::Result<()> {
    let std = arr.as_standard_layout();
    let mut f = File::create(path)?;
    write_header(&mut f, "<i2", std.shape())?;
    let mut bytes = Vec::with_capacity(std.len() * 2);
    for &v in std.iter() {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    f.write_all(&bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndarray::Array3;

    #[test]
    fn header_is_64_aligned_and_data_follows() {
        let a = Array3::<u16>::from_shape_fn((2, 3, 4), |(i, j, k)| (i * 100 + j * 10 + k) as u16);
        let path = std::env::temp_dir().join("vi_npy_test.npy");
        let p = path.to_str().unwrap();
        write_u16(p, &a).unwrap();
        let bytes = std::fs::read(p).unwrap();
        assert_eq!(&bytes[0..6], b"\x93NUMPY");
        let hlen = u16::from_le_bytes([bytes[8], bytes[9]]) as usize;
        // total header (magic+ver+len field + header) must be 64-aligned
        assert_eq!((10 + hlen) % 64, 0);
        // data length = 2*3*4 elements * 2 bytes
        assert_eq!(bytes.len(), 10 + hlen + 2 * 3 * 4 * 2);
        // first element little-endian == 0
        let off = 10 + hlen;
        assert_eq!(u16::from_le_bytes([bytes[off], bytes[off + 1]]), 0);
    }
}
```

- [ ] **Step 2: lib.rs にモジュール公開を追加**

`vi_ros2/vi_node/src/lib.rs` に追記 (既存の `pub mod ...` 群の末尾):

```rust
pub mod npy;
```

- [ ] **Step 3: テストが通ることを確認**

Run: `cd vi_ros2/vi_node && cargo test --lib --no-default-features npy::`
Expected: `header_is_64_aligned_and_data_follows` PASS。

- [ ] **Step 4: Commit**

```bash
git add vi_ros2/vi_node/src/npy.rs vi_ros2/vi_node/src/lib.rs
git commit -m "feat(vi_node): add dependency-free npy writer for value/policy dump"
```

### Task 2.2: DumpData + compute_policy + spawn_sweep に dump_slot (TDD)

**Files:**
- Modify: `vi_ros2/vi_node/src/sweep_thread.rs`

- [ ] **Step 1: 失敗するテストを追加**

`sweep_thread.rs` の `mod tests` に追加:

```rust
#[test]
fn dump_slot_is_filled_on_exit() {
    use std::sync::Mutex;
    let ctx = ctx_with_goal();
    let cancel = Arc::new(AtomicBool::new(false));
    let slot: Arc<Mutex<Option<DumpData>>> = Arc::new(Mutex::new(None));
    let h = spawn_sweep(ctx, Box::new(Reference { threshold: 0 }), cancel, Some(Arc::clone(&slot)));
    h.join.join().expect("worker panicked");
    let guard = slot.lock().unwrap();
    let dump = guard.as_ref().expect("dump slot must be filled");
    assert_eq!(dump.value.shape(), &[8, 8, vi_core::N_THETA]);
    assert_eq!(dump.policy.shape(), &[8, 8, vi_core::N_THETA]);
    // goal cell policy must be -1 (masked)
    // (Empty map goal is at center per generate_map; just assert range validity)
    for &v in dump.policy.iter() {
        assert!(v == -1 || (0..vi_core::N_ACTIONS as i16).contains(&v));
    }
}
```

- [ ] **Step 2: 実装 — DumpData / compute_policy / spawn_sweep シグネチャ変更**

`sweep_thread.rs` 冒頭の use を更新:

```rust
use std::sync::{Arc, Mutex};
use ndarray::{s, Array2, Array3};
use vi_core::{ActionIdx, Value, MAX_VALUE, N_THETA, PENALTY_OBSTACLE};
```

`FeedbackTick` の下に追加:

```rust
pub struct DumpData {
    pub value: Array3<Value>,
    pub policy: Array3<i16>,
}

/// Build the full optimal-policy table; -1 where obstacle / goal / unreachable
/// (mirrors the legacy node's `optimal_action_ == NULL ? -1`).
pub fn compute_policy(ctx: &VIContext) -> Array3<i16> {
    let h = ctx.dims.map_y as usize;
    let w = ctx.dims.map_x as usize;
    let mut pol = Array3::<i16>::from_elem((h, w, N_THETA), -1);
    for iy in 0..h {
        for ix in 0..w {
            if ctx.penalty[[iy, ix]] == PENALTY_OBSTACLE {
                continue;
            }
            for it in 0..N_THETA {
                if ctx.goal_mask[[iy, ix, it]] || ctx.value[[iy, ix, it]] == MAX_VALUE {
                    continue;
                }
                pol[[iy, ix, it]] =
                    vi_algorithm::optimal_action_at(ctx, ix as i32, iy as i32, it) as i16;
            }
        }
    }
    pol
}
```

`spawn_sweep` シグネチャに `dump_slot: Option<Arc<Mutex<Option<DumpData>>>>` を追加し、loop 終了直後に格納:

```rust
pub fn spawn_sweep(
    mut ctx: VIContext,
    solver: Box<dyn Solver>,
    cancel: Arc<AtomicBool>,
    dump_slot: Option<Arc<Mutex<Option<DumpData>>>>,
) -> SweepHandle {
    let (feedback_tx, feedback_rx) = unbounded::<FeedbackTick>();
    let (request_tx, request_rx) = unbounded::<WorkerRequest>();
    let cancel_inner = Arc::clone(&cancel);

    let join = thread::spawn(move || {
        let mut total: u32 = 0;
        let mut last_stats = SolveStats {
            iters_or_sweeps: 0,
            updates: 0,
            final_delta: vi_core::MAX_VALUE,
            converged: false,
            extra: None,
        };
        loop {
            while let Ok(req) = request_rx.try_recv() {
                match req {
                    WorkerRequest::ValueSlice { theta_idx, resp } => {
                        let slice = ctx.value.slice(s![.., .., theta_idx]).to_owned();
                        let _ = resp.send(slice);
                    }
                    WorkerRequest::OptimalAction { ix, iy, it, resp } => {
                        let a = vi_algorithm::optimal_action_at(&ctx, ix, iy, it);
                        let _ = resp.send(a);
                    }
                }
            }
            if cancel_inner.load(Ordering::Relaxed) { break; }
            let stats = solver.run(&mut ctx, Budget::Sweeps(1));
            total = total.saturating_add(stats.iters_or_sweeps);
            let _ = feedback_tx.send(FeedbackTick {
                sweep_count: total,
                final_delta: stats.final_delta,
            });
            let done = stats.converged;
            last_stats = stats;
            if done { break; }
        }
        if let Some(slot) = dump_slot {
            let policy = compute_policy(&ctx);
            *slot.lock().unwrap() = Some(DumpData { value: ctx.value.clone(), policy });
        }
        last_stats
    });

    SweepHandle { cancel, feedback_rx, request_tx, join }
}
```

- [ ] **Step 3: 既存のテスト呼び出しを 4 引数化**

`sweep_thread.rs` の `mod tests` 内の `spawn_sweep(...)` 呼び出し (4箇所: converges_and_joins / cancel_stops_worker / value_slice_request / optimal_action_request) に `, None` を追加。例:
`let h = spawn_sweep(ctx, Box::new(Reference { threshold: 0 }), cancel, None);`

- [ ] **Step 4: テストが通ることを確認**

Run: `cd vi_ros2/vi_node && cargo test --lib --no-default-features sweep_thread::`
Expected: 既存 4 件 + `dump_slot_is_filled_on_exit` が PASS。

- [ ] **Step 5: Commit**

```bash
git add vi_ros2/vi_node/src/sweep_thread.rs
git commit -m "feat(vi_node): worker fills value/policy DumpData on exit (gated by slot)"
```

### Task 2.3: main.rs に bench_dump_path + npy 書き出し

**Files:**
- Modify: `vi_ros2/vi_node/src/main.rs`

- [ ] **Step 1: Params に bench_dump_path を追加**

`struct Params { ... }` に `bench_dump_path: String,` を追加。`read_params` に宣言を追加 (他スカラーと同じ書式):

```rust
    let bench_dump_path = node
        .declare_parameter::<String>("bench_dump_path")
        .default("".to_string())
        .mandatory()
        .map_err(|e| anyhow!("declare bench_dump_path: {e}"))?
        .get();
```

`Ok(Params { ... })` の構築に `bench_dump_path,` を追加。

- [ ] **Step 2: action コールバックで dump slot を生成し spawn_sweep へ渡す**

`spawn_action_server` の冒頭クローン群に `let bench_dump_path = params.bench_dump_path.clone();` を追加し、`move` クロージャにも `let bench_dump_path = bench_dump_path.clone();` を取り込む。
Step 5 (spawn) を次に変更:

```rust
                let cancel = Arc::new(AtomicBool::new(false));
                let dump_slot = if bench_dump_path.is_empty() {
                    None
                } else {
                    Some(std::sync::Arc::new(std::sync::Mutex::new(
                        None::<vi_node::sweep_thread::DumpData>,
                    )))
                };
                let handle = spawn_sweep(base_ctx, solver, Arc::clone(&cancel), dump_slot.clone());
                let feedback_rx = handle.feedback_rx.clone();
```

- [ ] **Step 3: join 後 (step 7) に npy を書き出す**

step 7 の `let finished = ...;` の直後、`succeeded_with` の直前に:

```rust
                if let Some(slot) = dump_slot {
                    if let Some(dump) = slot.lock().unwrap().take() {
                        let vpath = format!("{}/value_ros2.npy", bench_dump_path);
                        let ppath = format!("{}/policy_ros2.npy", bench_dump_path);
                        if let Err(e) = vi_node::npy::write_u16(&vpath, &dump.value) {
                            eprintln!("ERROR: write {vpath}: {e}");
                        }
                        if let Err(e) = vi_node::npy::write_i16(&ppath, &dump.policy) {
                            eprintln!("ERROR: write {ppath}: {e}");
                        }
                        eprintln!("bench dump written to {bench_dump_path}");
                    }
                }
```

(`use std::sync::atomic::Ordering;` 等は既存。`dump_slot` は step 7 の join ブロック内で `sweep_handle` から取得するのではなく Step 2 で作ったものを `move` で捕捉している点に注意 — クロージャ内ローカルなので有効。)

- [ ] **Step 4: ビルド確認**

Run: `make ros2-build`
Expected: exit 0。型エラーが出た場合は dump_slot の `Arc<Mutex<Option<DumpData>>>` 型注釈を `vi_node::sweep_thread::DumpData` に明示。

- [ ] **Step 5: Commit**

```bash
git add vi_ros2/vi_node/src/main.rs
git commit -m "feat(vi_node): bench_dump_path param dumps full value/policy npy on result"
```

### Task 2.4: ROS2 vi_node パラメータ + rclpy client + 実行スクリプト

**Files:**
- Create: `vi_compare/ros2/vi_node_params.yaml`
- Create: `vi_compare/ros2/bench_client.py`
- Create: `vi_compare/ros2/run_ros2_bench.sh`

- [ ] **Step 1: vi_node_params.yaml (params.yaml :: planning をミラー)**

```yaml
# keep in sync with vi_compare/params.yaml :: planning
vi_node:
  ros__parameters:
    solver: "reference"
    theta_cell_num: 60
    safety_radius: 0.2
    safety_radius_penalty: 30
    goal_margin_radius: 0.3
    goal_margin_theta: 15.0
    online: false
    delta_threshold: 0
    thread_num: 1
    map_wait_sec: 60
    allow_action_mismatch: false
    bench_dump_path: "/results"
    action_names: ["forward","back","right","rightfw","left","leftfw"]
    action_forward_m: [0.3, -0.2, 0.0, 0.2, 0.0, 0.2]
    action_rotation_deg: [0.0, 0.0, -20.0, -20.0, 20.0, 20.0]
```

- [ ] **Step 2: rclpy bench_client.py**

```python
#!/usr/bin/env python3
"""ROS2 bench client: publish /map (house.pgm), send Vi goal, time to result.
value/policy are dumped by vi_node (bench_dump_path)."""
import sys, json, time, os
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from vi_interfaces.action import Vi

def load_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxv = int(f.readline())
        data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape((h, w))
    return w, h, data

def load_map_yaml(pgm_path):
    """Read the sibling <map>.yaml (map_server format) for geometry + thresholds."""
    yaml_path = os.path.splitext(pgm_path)[0] + '.yaml'
    with open(yaml_path) as f:
        m = yaml.safe_load(f)
    origin = m.get('origin', [0.0, 0.0, 0.0])
    return dict(resolution=float(m['resolution']),
                ox=float(origin[0]), oy=float(origin[1]),
                occupied_thresh=float(m.get('occupied_thresh', 0.65)),
                free_thresh=float(m.get('free_thresh', 0.196)),
                negate=int(m.get('negate', 0)))

def to_occupancy(w, h, pgm, meta):
    # map_server semantics (negate=0): occ_prob = (255 - p)/255
    # unknown (gray, e.g. pixel 205) -> occ_prob=0.196 which is NOT < free_thresh -> -1 (unknown)
    p = pgm.astype(np.float64)
    occ_prob = (p / 255.0) if meta['negate'] else ((255.0 - p) / 255.0)
    occ = np.full((h, w), -1, dtype=np.int8)              # default: unknown
    occ[occ_prob < meta['free_thresh']] = 0               # free
    occ[occ_prob > meta['occupied_thresh']] = 100         # occupied
    # ROS OccupancyGrid is row-major bottom-up (origin bottom-left) -> flip vertically
    occ = np.flipud(occ)
    msg = OccupancyGrid()
    msg.info.resolution = meta['resolution']
    msg.info.width = w
    msg.info.height = h
    msg.info.origin.position.x = meta['ox']
    msg.info.origin.position.y = meta['oy']
    msg.info.origin.orientation.w = 1.0
    msg.data = occ.reshape(-1).tolist()
    return msg

class BenchNode(Node):
    def __init__(self, p, map_msg):
        super().__init__('vi_bench_client_ros2')
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history = QoSHistoryPolicy.KEEP_LAST
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos)
        self.map_pub.publish(map_msg)
        self.ac = ActionClient(self, Vi, 'vi_controller')
        self.p = p
        self.elapsed = None
        self.sweeps = None
        self.converged = None

    def send(self):
        g = self.p['goal']
        if not self.ac.wait_for_server(timeout_sec=120):
            raise RuntimeError('vi_controller action server not available')
        goal = Vi.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(g['x'])
        ps.pose.position.y = float(g['y'])
        yaw = np.deg2rad(g['yaw_deg'])
        ps.pose.orientation.z = float(np.sin(yaw / 2))
        ps.pose.orientation.w = float(np.cos(yaw / 2))
        goal.goal = ps
        t0 = time.monotonic()
        self._last_sweep = 0
        fut = self.ac.send_goal_async(goal, feedback_callback=self._fb)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)
        self.elapsed = time.monotonic() - t0
        self.converged = bool(rfut.result().result.finished)
        self.sweeps = int(self._last_sweep)

    def _fb(self, fb):
        d = fb.feedback.current_sweep_times.data
        if d:
            self._last_sweep = max(d)

def main():
    params_path, map_path, out_dir = sys.argv[1], sys.argv[2], sys.argv[3]
    with open(params_path) as f:
        p = yaml.safe_load(f)
    w, h, pgm = load_pgm(map_path)
    meta = load_map_yaml(map_path)
    map_msg = to_occupancy(w, h, pgm, meta)
    rclpy.init()
    node = BenchNode(p, map_msg)
    node.send()
    os.makedirs(out_dir, exist_ok=True)
    timing = dict(elapsed_sec=node.elapsed, sweeps=node.sweeps,
                  converged=node.converged,
                  thread_num=p['planning']['thread_num'],
                  delta_threshold=p['client']['delta_threshold'], side='ros2')
    with open(os.path.join(out_dir, 'timing_ros2.json'), 'w') as f:
        json.dump(timing, f, indent=2)
    node.get_logger().info(f"ROS2 bench done: {timing}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- [ ] **Step 3: run_ros2_bench.sh**

```bash
#!/usr/bin/env bash
# Build workspace, launch vi_node with dump params, run rclpy client, shutdown.
# Expects mounts: /workspace (new repo), /src_value_iteration (本家 maps), /results
set -e
source /opt/ros/humble/setup.sh
source /ros2_rust_ws/install/local_setup.sh
cd /workspace
bash scripts/ros2_build.sh
source /workspace/vi_ros2_ws/install/local_setup.sh
ros2 run vi_node vi_node --ros-args \
  --params-file /workspace/vi_compare/ros2/vi_node_params.yaml &
NODE=$!
trap 'kill $NODE 2>/dev/null || true' EXIT
sleep 3
python3 /workspace/vi_compare/ros2/bench_client.py \
  /workspace/vi_compare/params.yaml \
  /src_value_iteration/maps/house.pgm \
  /results
# give vi_node a moment to finish writing npy after returning result
sleep 2
```

- [ ] **Step 4: 構文チェック + ROS2 側単体実行**

Run: `python3 -m py_compile vi_compare/ros2/bench_client.py`
Expected: 構文 OK。

Run:
```bash
docker run --rm \
  -v $(pwd):/workspace \
  -v /home/nop/dev/mywork/value_iteration:/src_value_iteration:ro \
  -v $(pwd)/vi_compare/results:/results \
  vi_ros2_dev:humble \
  bash /workspace/vi_compare/ros2/run_ros2_bench.sh
```
Expected: `vi_compare/results/value_ros2.npy` (uint16, (384,384,60))・`policy_ros2.npy` (int16)・`timing_ros2.json` が生成。`converged: true`。

- [ ] **Step 5: npy 健全性確認**

Run:
```bash
python3 -c "import numpy as np; v=np.load('vi_compare/results/value_ros2.npy'); print(v.shape,v.dtype,(v<65535).sum(),'reachable')"
```
Expected: `(384,384,60) uint16`、可達セル数が 0 でない。

- [ ] **Step 6: Commit**

```bash
git add vi_compare/ros2/vi_node_params.yaml vi_compare/ros2/bench_client.py vi_compare/ros2/run_ros2_bench.sh
git commit -m "feat(vi_compare): add ROS2 rclpy bench client + runner + node params"
```

---

## Phase 3 — compare.py + オーケストレータ + Makefile

### Task 3.1: compare.py コア関数 (TDD)

**Files:**
- Create: `vi_compare/compare/compare.py`
- Create: `vi_compare/compare/test_compare.py`

- [ ] **Step 1: 失敗するテストを書く**

`vi_compare/compare/test_compare.py`:

```python
#!/usr/bin/env python3
import numpy as np
import compare as C

def test_orientation_recovers_transpose():
    rng = np.random.default_rng(0)
    H = W = 6; T = 3
    ros2 = rng.integers(0, 50, size=(H, W, T)).astype(np.float64)
    unreach2 = np.zeros((H, W, T), bool)
    unreach2[0, :, :] = True  # a distinctive border
    ros2[unreach2] = 65535
    # ros1 is ros2 spatially transposed + value sentinel 1e9
    ros1 = np.transpose(ros2.copy(), (1, 0, 2))
    ros1[ros1 >= 65535] = 1e9
    aligned, name = C.align(ros1, ros2, ros1_unreach=ros1 >= 1e6, ros2_unreach=ros2 >= 65535)
    assert name == 'transpose', name
    # after alignment the unreachable borders coincide
    assert ((aligned >= 1e6) == (ros2 >= 65535)).mean() > 0.99

def test_value_metrics_identity():
    H = W = 5; T = 2
    a = np.arange(H * W * T, dtype=np.float64).reshape(H, W, T)
    m = C.value_metrics(a, a, reach=np.ones((H, W, T), bool))
    assert abs(m['rmse']) < 1e-9
    assert abs(m['pearson'] - 1.0) < 1e-9

def test_policy_agreement():
    a = np.array([[[0, 1], [2, -1]]], dtype=np.float64)   # shape (1,2,2)
    b = np.array([[[0, 3], [2, -1]]], dtype=np.float64)
    # valid cells (both>=0): (0,0,0)=0==0 ok, (0,0,1)=1!=3, (0,1,0)=2==2 ok ; (0,1,1) excluded(-1)
    assert abs(C.policy_agreement(a, b) - (2 / 3)) < 1e-9

if __name__ == '__main__':
    test_orientation_recovers_transpose()
    test_value_metrics_identity()
    test_policy_agreement()
    print("OK")
```

- [ ] **Step 2: テストを実行して失敗を確認**

Run: `cd vi_compare/compare && python3 test_compare.py`
Expected: FAIL (`ModuleNotFoundError: compare` か `AttributeError`)。

- [ ] **Step 3: compare.py を実装**

```python
#!/usr/bin/env python3
"""Align ROS1/ROS2 value & policy dumps and emit a comparison report."""
import sys, json, os
import numpy as np

ROS2_UNREACH = 65535
ROS1_UNREACH = 1e6

# 8 dihedral spatial transforms on the (H, W) plane (theta axis preserved).
# Order matters: on a tie in the unreachable-mask score the first wins (strict <),
# so simple/natural transforms precede rotations (transpose must beat rot270 on a
# row-symmetric mask).
_TRANSFORMS = {
    'identity':      lambda a: a,
    'fliplr':        lambda a: a[:, ::-1, :],
    'flipud':        lambda a: a[::-1, :, :],
    'transpose':     lambda a: np.transpose(a, (1, 0, 2)),
    'antitranspose': lambda a: np.transpose(a, (1, 0, 2))[::-1, ::-1, :],
    'rot90':         lambda a: np.rot90(a, 1, axes=(0, 1)),
    'rot180':        lambda a: np.rot90(a, 2, axes=(0, 1)),
    'rot270':        lambda a: np.rot90(a, 3, axes=(0, 1)),
}

def align(ros1, ros2, ros1_unreach, ros2_unreach):
    """Find spatial transform of ros1 that best matches ros2's unreachable mask.
    Returns (transformed_ros1, transform_name)."""
    best_name, best_disagree = 'identity', 1.0
    scores = {}
    for name, fn in _TRANSFORMS.items():
        if fn(ros1_unreach).shape != ros2_unreach.shape:
            continue
        disagree = (fn(ros1_unreach) != ros2_unreach).mean()
        scores[name] = disagree
        if disagree < best_disagree:
            best_disagree, best_name = disagree, name
    # sanity: best should be clearly better than 2nd best (unless near-perfect)
    ordered = sorted(scores.values())
    if len(ordered) > 1 and best_disagree > 0.02 and (ordered[1] - ordered[0]) < 0.01:
        print(f"WARN: ambiguous orientation (scores={scores})", file=sys.stderr)
    return _TRANSFORMS[best_name](ros1), best_name

def value_metrics(ros1, ros2, reach):
    a = ros1[reach].astype(np.float64)
    b = ros2[reach].astype(np.float64)
    n = a.size
    if n == 0:
        return dict(n=0, rmse=float('nan'), mae=float('nan'),
                    max_abs=float('nan'), pearson=float('nan'), spearman=float('nan'))
    diff = a - b
    rmse = float(np.sqrt(np.mean(diff ** 2)))
    mae = float(np.mean(np.abs(diff)))
    max_abs = float(np.max(np.abs(diff)))
    pearson = float(np.corrcoef(a, b)[0, 1]) if n > 1 and a.std() > 0 and b.std() > 0 else float('nan')
    ra = np.argsort(np.argsort(a))
    rb = np.argsort(np.argsort(b))
    spearman = float(np.corrcoef(ra, rb)[0, 1]) if n > 1 else float('nan')
    return dict(n=int(n), rmse=rmse, mae=mae, max_abs=max_abs,
                pearson=pearson, spearman=spearman)

def policy_agreement(pol1, pol2):
    valid = (pol1 >= 0) & (pol2 >= 0)
    if valid.sum() == 0:
        return float('nan')
    return float((pol1[valid] == pol2[valid]).mean())

def main():
    out_dir = sys.argv[1]
    v1 = np.load(os.path.join(out_dir, 'value_ros1.npy')).astype(np.float64)
    v2 = np.load(os.path.join(out_dir, 'value_ros2.npy')).astype(np.float64)
    p1 = np.load(os.path.join(out_dir, 'policy_ros1.npy')).astype(np.float64)
    p2 = np.load(os.path.join(out_dir, 'policy_ros2.npy')).astype(np.float64)

    u1 = v1 >= ROS1_UNREACH
    u2 = v2 >= ROS2_UNREACH
    v1a, tname = align(v1, v2, u1, u2)
    # apply the SAME transform to policy and unreachable mask
    p1a = _TRANSFORMS[tname](p1)
    u1a = _TRANSFORMS[tname](u1)

    reach = (~u1a) & (~u2)
    vm = value_metrics(v1a, v2, reach)
    pa_all = policy_agreement(p1a, p2)
    pa_t0 = policy_agreement(p1a[:, :, 0:1], p2[:, :, 0:1])

    t1 = json.load(open(os.path.join(out_dir, 'timing_ros1.json')))
    t2 = json.load(open(os.path.join(out_dir, 'timing_ros2.json')))

    lines = []
    lines.append("# VI 比較レポート (本家ROS1 vs vi_ros2 ROS2)\n")
    lines.append(f"- 整列変換 (ROS1→ROS2): **{tname}**")
    lines.append(f"- unreachable 一致率: {(u1a == u2).mean()*100:.2f}%  (整列の妥当性指標)\n")
    lines.append("## 速度\n")
    lines.append("| 側 | elapsed[s] | sweeps | converged | threads |")
    lines.append("|---|---|---|---|---|")
    lines.append(f"| ROS1(本家) | {t1['elapsed_sec']:.3f} | {t1['sweeps']} | {t1['converged']} | {t1['thread_num']} |")
    lines.append(f"| ROS2(vi_node) | {t2['elapsed_sec']:.3f} | {t2['sweeps']} | {t2['converged']} | {t2['thread_num']} |")
    speedup = (t1['elapsed_sec'] / t2['elapsed_sec']) if t2['elapsed_sec'] else float('nan')
    lines.append(f"\n- 速度比 (ROS1/ROS2): **{speedup:.2f}x**\n")
    lines.append("## 価値一致度 (両者可達セルのみ, ステップ単位)\n")
    lines.append(f"- 対象セル数: {vm['n']}")
    lines.append(f"- RMSE: {vm['rmse']:.4f},  MAE: {vm['mae']:.4f},  最大差: {vm['max_abs']:.4f}")
    lines.append(f"- Pearson: {vm['pearson']:.4f},  Spearman: {vm['spearman']:.4f}\n")
    lines.append("## 方策一致度 (両者可達セルのみ)\n")
    lines.append(f"- 全 theta: {pa_all*100:.2f}%")
    lines.append(f"- theta=0 スライス: {pa_t0*100:.2f}%")

    report = "\n".join(lines) + "\n"
    with open(os.path.join(out_dir, 'report.md'), 'w') as f:
        f.write(report)
    print(report)

if __name__ == '__main__':
    main()
```

- [ ] **Step 4: テストが通ることを確認**

Run: `cd vi_compare/compare && python3 test_compare.py`
Expected: `OK` (3 アサート全通過)。

- [ ] **Step 5: Commit**

```bash
git add vi_compare/compare/compare.py vi_compare/compare/test_compare.py
git commit -m "feat(vi_compare): add comparison core (align/value/policy) with tests"
```

### Task 3.2: docker-compose + results ディレクトリ

**Files:**
- Create: `vi_compare/docker/docker-compose.yml`
- Create: `vi_compare/results/.gitkeep`

- [ ] **Step 1: docker-compose.yml を作成**

```yaml
# Sequential orchestration is done by scripts/compare_bench.sh (not `up`),
# this file only fixes image/volume wiring for `docker compose run`.
services:
  ros1:
    image: vi_compare_ros1:noetic
    build:
      context: ..
      dockerfile: docker/Dockerfile.ros1
    volumes:
      - ${VI_ORIG:-../../../value_iteration}:/src_value_iteration:ro
      - ../../:/workspace
      - ../results:/results
    working_dir: /workspace
    command: bash /workspace/vi_compare/ros1/run_ros1_bench.sh

  ros2:
    image: vi_ros2_dev:humble
    volumes:
      - ${VI_ORIG:-../../../value_iteration}:/src_value_iteration:ro
      - ../../:/workspace
      - ../results:/results
    working_dir: /workspace
    command: bash /workspace/vi_compare/ros2/run_ros2_bench.sh
```

- [ ] **Step 2: results/.gitkeep**

```bash
mkdir -p vi_compare/results && touch vi_compare/results/.gitkeep
echo "*.npy" > vi_compare/results/.gitignore
echo "report.md" >> vi_compare/results/.gitignore
echo "timing_*.json" >> vi_compare/results/.gitignore
echo "!.gitkeep" >> vi_compare/results/.gitignore
```

- [ ] **Step 3: Commit**

```bash
git add vi_compare/docker/docker-compose.yml vi_compare/results/.gitkeep vi_compare/results/.gitignore
git commit -m "build(vi_compare): add docker-compose wiring + results dir"
```

### Task 3.3: オーケストレータ

**Files:**
- Create: `scripts/compare_bench.sh`

- [ ] **Step 1: compare_bench.sh を作成**

```bash
#!/usr/bin/env bash
# Sequential ROS1 → ROS2 → compare. Run from repo root.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ORIG="${VI_ORIG:-$(cd "$REPO_ROOT/.." && pwd)/value_iteration}"
RESULTS="$REPO_ROOT/vi_compare/results"
mkdir -p "$RESULTS"

echo "== [1/3] ROS1 (本家) =="
docker run --rm \
  -v "$ORIG":/src_value_iteration:ro \
  -v "$REPO_ROOT":/workspace \
  -v "$RESULTS":/results \
  vi_compare_ros1:noetic \
  bash /workspace/vi_compare/ros1/run_ros1_bench.sh

echo "== [2/3] ROS2 (vi_node) =="
docker run --rm \
  -v "$ORIG":/src_value_iteration:ro \
  -v "$REPO_ROOT":/workspace \
  -v "$RESULTS":/results \
  vi_ros2_dev:humble \
  bash /workspace/vi_compare/ros2/run_ros2_bench.sh

echo "== [3/3] compare =="
docker run --rm \
  -v "$REPO_ROOT":/workspace \
  -v "$RESULTS":/results \
  vi_compare_ros1:noetic \
  bash -lc "cd /workspace/vi_compare/compare && python3 compare.py /results"

echo "report: $RESULTS/report.md"
```

- [ ] **Step 2: 実行権付与 + Commit**

```bash
chmod +x scripts/compare_bench.sh vi_compare/ros1/run_ros1_bench.sh vi_compare/ros2/run_ros2_bench.sh
git add scripts/compare_bench.sh
git update-index --chmod=+x scripts/compare_bench.sh vi_compare/ros1/run_ros1_bench.sh vi_compare/ros2/run_ros2_bench.sh
git commit -m "feat(vi_compare): add sequential orchestrator script"
```

### Task 3.4: Makefile ターゲット

**Files:**
- Modify: `Makefile` (148行目 `.PHONY: ros2-*` の直後)

- [ ] **Step 1: compare-* ターゲットを追加**

`Makefile` の `.PHONY: ros2-docker ros2-shell ros2-build ros2-test` 行の直後に追記:

```makefile

# ----- vi_compare (本家ROS1 vs vi_ros2 ROS2 ベンチ) -------------------

VI_ORIG ?= $(abspath $(PWD)/../value_iteration)

compare-build: ros2-docker
	docker build -t vi_compare_ros1:noetic -f vi_compare/docker/Dockerfile.ros1 vi_compare/docker

compare-ros1:
	docker run --rm \
	  -v $(VI_ORIG):/src_value_iteration:ro \
	  -v $(PWD):/workspace -v $(PWD)/vi_compare/results:/results \
	  vi_compare_ros1:noetic bash /workspace/vi_compare/ros1/run_ros1_bench.sh

compare-ros2:
	docker run --rm \
	  -v $(VI_ORIG):/src_value_iteration:ro \
	  -v $(PWD):/workspace -v $(PWD)/vi_compare/results:/results \
	  vi_ros2_dev:humble bash /workspace/vi_compare/ros2/run_ros2_bench.sh

compare-report:
	docker run --rm -v $(PWD):/workspace -v $(PWD)/vi_compare/results:/results \
	  vi_compare_ros1:noetic bash -lc "cd /workspace/vi_compare/compare && python3 compare.py /results"

compare-bench: compare-build
	VI_ORIG=$(VI_ORIG) bash scripts/compare_bench.sh

.PHONY: compare-build compare-ros1 compare-ros2 compare-report compare-bench
```

- [ ] **Step 2: ターゲット存在確認**

Run: `make -n compare-bench`
Expected: `compare-build` 経由でビルド→`scripts/compare_bench.sh` 呼び出しのコマンドが表示される (構文エラーなし)。

- [ ] **Step 3: Commit**

```bash
git add Makefile
git commit -m "build(vi_compare): add compare-bench Make targets"
```

---

## Phase 4 — 統合実走 & 調整

### Task 4.1: ゴール座標の確定 (free セル)

**Files:**
- Modify: `vi_compare/params.yaml`

- [ ] **Step 1: house.pgm の free 領域から妥当なゴールを選ぶ**

Run:
```bash
python3 - <<'PY'
import numpy as np
with open('/home/nop/dev/mywork/value_iteration/maps/house.pgm','rb') as f:
    f.readline(); l=f.readline()
    while l.startswith(b'#'): l=f.readline()
    w,h=map(int,l.split()); f.readline()
    img=np.frombuffer(f.read(w*h),dtype=np.uint8).reshape(h,w)
free=img>166
# world coords: origin [-10,-10], res 0.05, ROS y is bottom-up
ys,xs=np.where(np.flipud(free))
# pick a central free cell
cx,cy=int(np.median(xs)),int(np.median(ys))
print("goal world x,y =", -10+0.05*cx, -10+0.05*cy)
PY
```
Expected: free セル中心付近の `(x, y)` 世界座標が出力される。

- [ ] **Step 2: params.yaml の goal を確定値に更新**

Step1 の出力で `vi_compare/params.yaml` の `goal.x` / `goal.y` を置き換える (yaw_deg は 0.0 のまま)。

- [ ] **Step 3: Commit**

```bash
git add vi_compare/params.yaml
git commit -m "chore(vi_compare): set concrete goal at a free cell of house.pgm"
```

### Task 4.2: フルベンチ実行 & レポート確認

**Files:** (なし。実走)

- [ ] **Step 1: フル実行**

Run: `make compare-bench`
Expected: 3 段 (ROS1→ROS2→compare) が逐次成功し、`vi_compare/results/report.md` が生成。

- [ ] **Step 2: レポート健全性チェック**

Run: `cat vi_compare/results/report.md`
Expected:
- 整列変換が決まり、**unreachable 一致率 > 95%** (整列が正しい)。
- 速度表に両側の elapsed/sweeps が入る。
- 価値: 対象セル数 > 0、Spearman 相関が高い (> 0.9 目安。確率遷移/ペナルティ差で RMSE はゼロにならない)。
- 方策一致率が算出される。

- [ ] **Step 3: 整列が ambiguous または unreachable 一致率が低い場合の調整**

- compare.py の WARN が出る/一致率が低い → `bench_client.py(ros1)` の `decode_gridmap` の order や transpose を見直す (grid_map の column-major と `map.at(Index(ix,iy))` の対応)。`align()` の候補変換で補正されるはずだが、補正後も低い場合は OccupancyGrid の y 反転 (`np.flipud`) の有無を ROS2 側 `to_occupancy` と突き合わせる。
- 収束しない (`converged: false`) → `params.yaml` の `delta_threshold` を 1 に緩和、または `max_sweeps`/`timeout_sec` を拡大。

- [ ] **Step 4: 最終コミット (生成物は .gitignore 済み、スクリプト微修正があれば)**

```bash
git add -A vi_compare scripts
git commit -m "chore(vi_compare): tune convergence/alignment after full run" || echo "no changes"
```

---

## 受け入れ基準 (spec §9 と対応)

- [ ] `make compare-bench` 一発で両コンテナ逐次起動 → `vi_compare/results/report.md` 生成。
- [ ] report に (1) 速度表 (2) 価値一致度 (RMSE/相関) (3) 方策一致率 が含まれる。
- [ ] 同一設定で再実行時、結果指標が再現 (計時はぶれる旨を report 外で注記)。
- [ ] 本家リポジトリ (`../value_iteration`) に変更なし (ro マウント)。
- [ ] vi_node の本番挙動 (`bench_dump_path` 未設定時) が改修前と不変 (Phase 2 のユニットテスト + `cargo test --lib` で担保)。

## Self-Review メモ (spec coverage)

- spec §3 (ROS1) → Phase 1 (Task 1.1-1.5)
- spec §4 (ROS2 + vi_node 改修) → Phase 0 (ビルド) + Phase 2 (Task 2.1-2.4)
- spec §5 (公平性/パラメータ) → `params.yaml` / `bench.launch` / `vi_node_params.yaml` (Task 1.2/1.3/2.4) + Task 4.1
- spec §6 (比較指標/出力) → Task 3.1
- spec §7 (実行方法) → Task 3.3/3.4
- spec §8 (リスク/段階) → Phase 0 を最初に配置
- 型整合: `DumpData{value:Array3<u16>, policy:Array3<i16>}` を Task 2.1/2.2/2.3 で一貫使用。npy writer `write_u16`/`write_i16`。compare.py の `align/value_metrics/policy_agreement` を Task 3.1 で定義・テスト。
