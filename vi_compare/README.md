# vi_compare — 本家 ROS1 `value_iteration` との比較ベンチ一式

本家 ROS1 ノード (`../value_iteration`、読み取り専用マウント) と
このリポジトリの実装 (vi_rs / vi_ros2) を、同一問題・同一パラメータで
突き合わせるためのハーネス群。**本家リポジトリは一切改変しない**
(パッチが要る場合は `video/snapshot.patch` のようにコンテナ内のコピーへ適用する)。

## ディレクトリ構成

```
vi_compare/
├── docker/                  ROS1 noetic ベンチイメージ (Dockerfile.ros1)
├── benches/
│   ├── house/               house マップでのオラクル比較スイート (単スレ中心)
│   │   ├── params.yaml      共通パラメータ (goal, theta 数など)
│   │   ├── ros1/            本家ノードを action 経由で叩くベンチ
│   │   ├── vi_rs/           ROS 非経由の Rust 直接ハーネス
│   │   │                    (ref_bench = vi_reference 忠実移植単体,
│   │   │                     u64_bench = u64 高速ソルバ群一括)
│   │   └── compare/         npy 突き合わせ・レポート生成 (compare.py ほか)
│   └── tsudanuma/           論文 (Ueda+ 2023) 構成の並列スイープ再現スイート
│       ├── maps/            crop_pool.py / embed_fullext.py (lite/full 地図生成;
│       │                    プールは tsukuba/maps/pool_tsukuba.py を共用)
│       ├── ros1/            bench_client_tsudanuma.py (tsukuba と共用) +
│       │                    run_sweep_ros1_reps.sh (m掃引; REPS=1 で単発)
│       ├── vi_rs/           run_sweep_vi_rs_reps.sh (bench_map の m 掃引; host 実行)
│       ├── plot.py          図生成 (speed/house/lite/full/tsukuba モード)
│       └── report_paper_style.md              論文対応レポート
│   └── tsukuba/             tsukuba フル地図 (226M states) の動画素材生成
│       └── ros1/            bench_tsukuba.launch (tsudanuma 系ベンチと共用) +
│                            run_snap_tsukuba.sh (snapshot 付き TIMEOUT ラン)
├── video/                   スイープ可視化動画のレンダラ
│   ├── render_frames.py         house / tsudanuma / tsukuba 共通レンダラ
│   │                            (house = ×8 スローモーション, 他 = real-time → ×40 timelapse)
│   └── snapshot.patch           本家へ snapshotWorker を足すパッチ (コピーに適用)
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
└── tsukuba/        tsukuba マップ (2650×1420×60 = 226M states, scale5/0.25m)
    ├── map_tsukuba_pooled.pgm/.yaml  pooled 地図 (origin -553.84/-60.609)
    ├── value.bin path.bin fig_map_overlay_tsukuba.png  PoC (sparse 6.9s 収束)
    ├── frames_sparse/ frames_ros1/ snap_run/  動画素材 (計 ~6GB、消去可)
    └── video_ros1_vs_virs_tsukuba_full.mp4   ROS1 vs vi_rs 速度比較動画
        (real-time → ×40 timelapse; vi_rs 6.9s 厳密収束 vs ROS1 575s 未収束)
```

ディスクを空けたいとき: `frames_*/`, `video_frames/`, `snap_run/` は動画の
中間データなので削除してよい (再生成にはスナップショット付き再計測が必要)。

## 実行方法

### house オラクル比較 (リポジトリルートの Makefile から)

```sh
make compare-build          # ROS1 noetic イメージ
make compare-bench          # 本家 → ref → 突き合わせを一括 (scripts/compare_bench.sh)
make compare-u64 compare-u64-summary   # u64 全ソルバ + report_u64.md
make compare-strict         # 真の固定点での bit 比較 (scripts/compare_strict.sh)
```

### 津田沼 / 並列スイープ

`benches/tsudanuma/ros1/` の `run_sweep_ros1_reps.sh`
(スレッド数掃引; 単発は REPS=1) を docker (`vi_compare_ros1:noetic`) 内で、
`vi_rs/run_sweep_vi_rs_reps.sh` を host で実行。env (`MAP_YAML`, `OUTDIR`,
`GOAL_*`, `MLIST`, `DELTA_THR`, `TIMEOUT`, `SOLVER` など) で制御。
図は `plot.py [RES] [mode]` (mode = all/speed/house/lite/full/tsukuba,
RES 省略時 `../../results/tsudanuma`;
matplotlib は docker イメージ `raspicat-vla-sim:latest` で実行)。

### 動画

1. スナップショット付き計測: vi_rs 側は `VI_SNAP_DIR`/`VI_SNAP_EVERY` env
   (frontier2d_sparse の Snapshotter)、本家側は無改変の本家 checkout を
   `/src_value_iteration` にマウントし `VI_SNAP_DIR`/`VI_SNAP_MS` env
   (run_snap_tsukuba.sh がコピーへ `video/snapshot.patch` を適用する)。
2. レンダリング: `video/render_frames.py {house,tsudanuma,tsukuba}` を raspicat-vla-sim で実行
   (リポジトリを `/work` にマウント) → `results/<map>/video_frames/` に PNG。
3. エンコード: host ffmpeg は libx264 無し — `h264_nvenc -cq 21` か
   `libopenh264` を使う。

tsukuba フル動画の再現 (0.15m/scale3 = 4417×2367×60 = 627M states; 津田沼と同設定
goal world(20.5,-1.0,0°), margin_theta=15, radius=0.30。0.15m では goal mask=28
セルで孤立しないので margin=15 のまま両側を揃えられる。0.25m 版が使った
margin=180 回避策は不要)。**ピーク ~46GB 要 (states 35GB + Fused 11GB)** なので
128GB 機推奨。RAM が足りない機では `VI_SNAP_DROP_STATES=1` で states を解放しつつ
snapshot を出せる (write_back/policy は skip)。純 sweep ~17s で厳密収束:

```sh
# vi_rs sparse スナップショット (~17s 収束 + dump。pooled.yaml は 0.15m なので --scale 1)
VI_THREADS=16 VI_SNAP_DIR=vi_compare/results/tsukuba/frames_sparse VI_SNAP_EVERY=5 \
  vi_rs/target/release/bench_map \
  --map vi_compare/results/tsukuba/map_tsukuba_pooled.yaml --scale 1 \
  --solver frontier2d_sparse --goal-x=20.5 --goal-y=-1.0 --goal-theta-deg=0 \
  --goal-radius-m 0.30 --goal-margin-theta-deg 15 \
  --safety-radius-m 0.20 --safety-penalty 100000 --unknown obstacle

# 本家 ROS1 スナップショット (docker, TIMEOUT まで未収束)
docker run --rm \
  -v "$PWD/../value_iteration:/src_value_iteration:ro" \
  -v "$PWD:/workspace" -v "$PWD/vi_compare/.cache/catkin_ws:/catkin_ws" \
  -e VI_SNAP_MS=2000 -e TIMEOUT=600 vi_compare_ros1:noetic \
  bash /workspace/vi_compare/benches/tsukuba/ros1/run_snap_tsukuba.sh

# レンダ + エンコード
docker run --rm -v "$PWD:/work" raspicat-vla-sim:latest \
  python3 /work/vi_compare/video/render_frames.py tsukuba
cd vi_compare/results/tsukuba && ffmpeg -y -framerate 30 \
  -i video_frames/frame_%05d.png -c:v libx264 -crf 20 -preset medium -pix_fmt yuv420p \
  video_ros1_vs_virs_tsukuba_015.mp4   # libx264 無い環境は -c:v h264_nvenc -cq 21
```

## 整合性の注意

- goal 円盤計算は origin 依存 (本家 `setStateValues`)。両側で同一 origin なら可:
  津田沼は origin (0,0) で揃える。tsukuba は実 origin (-553.84/-60.609) を両側で
  共有 (vi_rs/ROS1 とも同一 yaml; goal cell が一致するので比較は等価)。
- 本家 feedback の `_delta` は 18bit 固定小数の生値 (PROB_BASE=262144 = 1 s)。
- `bench_map` の負座標は `--goal-y=-2.0` 形式 (clap)。
