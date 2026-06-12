# vi_compare — 本家 ROS1 `value_iteration` との比較ベンチ一式

本家 ROS1 ノード (`~/dev/mywork/value_iteration`、読み取り専用マウント) と
このリポジトリの実装 (vi_rs / vi_ros2) を、同一問題・同一パラメータで
突き合わせるためのハーネス群。**本家リポジトリは一切改変しない**
(パッチが要る場合は `video/value_iteration_snap/` のようにコピーへ適用する)。

## ディレクトリ構成

```
vi_compare/
├── docker/                  ROS1 noetic ベンチイメージ (Dockerfile.ros1, compose)
├── benches/
│   ├── house/               house マップでのオラクル比較スイート (単スレ中心)
│   │   ├── params.yaml      共通パラメータ (goal, theta 数など)
│   │   ├── ros1/  ros2/     本家ノード / vi_node を action 経由で叩くベンチ
│   │   ├── vi_rs/           ROS 非経由の Rust 直接ハーネス
│   │   │                    (ref_bench = vi_reference 忠実移植単体,
│   │   │                     u64_bench = u64 高速ソルバ群一括)
│   │   └── compare/         npy 突き合わせ・レポート生成 (compare.py ほか)
│   └── tsudanuma/           論文 (Ueda+ 2023) 構成の並列スイープ再現スイート
│       ├── maps/            pool_tsudanuma.py / crop_pool.py (lite/full 地図生成)
│       ├── ros1/            bench_tsudanuma.launch + bench_client +
│       │                    run_bench.sh (フル収束計測) / run_sweep_ros1.sh (m掃引)
│       │                    / parse_ros1_log.py (手動停止ラン救済)
│       ├── vi_rs/           run_sweep_vi_rs.sh (bench_map の m 掃引; host 実行)
│       ├── plot.py          図生成 (speed/house/lite/full/tsukuba モード)
│       └── report_paper_style.md              論文対応レポート
├── video/                   スイープ可視化動画のレンダラ
│   ├── render_frames.py         house 版 (×8 スローモーション)
│   ├── render_frames_full.py    津田沼フル版 (real-time → ×40 timelapse)
│   └── value_iteration_snap/    本家のパッチ済コピー (snapshotWorker 追加)
├── results/                 生成物 (git 管理外; 下記)
└── .cache/                  catkin_ws / cargo target の永続キャッシュ (git 管理外)
```

## results/ の中身 (すべて再生成可能・コミットしない)

```
results/
├── house_oracle/   benches/house の出力: value_/policy_<solver>.npy,
│                   timing_*.json, report*.md (本家との bit-exact 検証)
├── house/          並列スレッド掃引 (sweep_*.csv) + 動画素材
│   ├── frames_ros1/ frames_sparse/   スナップショット (f32 min-θ 値場 + times.csv)
│   ├── snap_run/                     スナップショット付き計測ランのログ
│   ├── video_frames/                 レンダラ出力 PNG (派生物・消去可)
│   └── video_ros1_vs_virs_house.mp4
├── tsudanuma/      lite/ (540×540 論文規模) と full/ (1963×1334) の成果一式
│   ├── sweep_*.csv ros1_parallel.*   スレッド掃引・収束計測
│   ├── lite/  full/                  pooled 地図 + 値場 bin + 図
│   ├── frames_ros1/ frames_sparse/ snap_run/  動画素材 (計 ~5GB、消去可)
│   ├── logs/                         生ログ
│   └── video_ros1_vs_virs_tsudanuma_full.mp4
└── tsukuba/        tsukuba マップ PoC (scale5, sparse 7.1s)
```

ディスクを空けたいとき: `frames_*/`, `video_frames/`, `snap_run/` は動画の
中間データなので削除してよい (再生成にはスナップショット付き再計測が必要)。

## 実行方法

### house オラクル比較 (リポジトリルートの Makefile から)

```sh
make compare-build          # ROS1 noetic イメージ
make compare-ros1           # 本家を実行 → /results (= results/house_oracle) に npy
make compare-ref            # vi_reference u64 ハーネス
make compare-u64 compare-u64-summary   # u64 全ソルバ + report_u64.md
make compare-report         # ros1 vs ros2 突き合わせ
```

(`scripts/compare_bench.sh` / `scripts/compare_strict.sh` も同じ配置を使う)

### 津田沼 / 並列スイープ

`benches/tsudanuma/ros1/` の `run_bench.sh` (フル収束計測) と
`run_sweep_ros1.sh` (スレッド数掃引) を docker (`vi_compare_ros1:noetic`) 内で、
`vi_rs/run_sweep_vi_rs.sh` を host で実行。env (`MAP_YAML`, `OUTDIR`,
`GOAL_*`, `MLIST`, `DELTA_THR`, `TIMEOUT`, `SOLVER` など) で制御。
図は `plot.py [RES] [mode]` (mode = all/speed/house/lite/full/tsukuba,
RES 省略時 `../../results/tsudanuma`;
matplotlib は docker イメージ `raspicat-vla-sim:latest` で実行)。

### 動画

1. スナップショット付き計測: vi_rs 側は `VI_SNAP_DIR`/`VI_SNAP_EVERY` env
   (frontier2d_sparse の Snapshotter)、本家側は `video/value_iteration_snap/`
   を `/src_value_iteration` にマウントし `VI_SNAP_DIR`/`VI_SNAP_MS` env。
2. レンダリング: `video/render_frames{,_full}.py` を raspicat-vla-sim で実行
   (リポジトリを `/work` にマウント) → `results/<map>/video_frames/` に PNG。
3. エンコード: host ffmpeg は libx264 無し — `h264_nvenc -cq 21` か
   `libopenh264` を使う。

## 整合性の注意

- pooled 地図の origin は (0,0) 必須 (本家 `setStateValues` の goal 円盤計算が
  origin に依存; `bench_map::build_occupancy` と bit 一致させる)。
- 本家 feedback の `_delta` は 18bit 固定小数の生値 (PROB_BASE=262144 = 1 s)。
- `bench_map` の負座標は `--goal-y=-2.0` 形式 (clap)。
