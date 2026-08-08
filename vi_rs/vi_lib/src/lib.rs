//! 本家 ROS1 `value_iteration` パッケージ (`ValueIterator` / `ValueIteratorLocal`) の
//! Rust 忠実移植。型・アルゴリズム・固有バグまで一致させることを目的とする。
//! 設計: `docs/superpowers/specs/2026-06-08-vi-reference-faithful-port-design.md`

pub mod params;
pub mod msg;

pub use msg::{LaserScan, OccupancyGrid, Quaternion};
pub mod state_transition;

pub use state_transition::StateTransition;
pub mod action;

pub use action::Action;

// 本家 SweepWorkerStatus は本家ソルバ機構ごと solvers::original へ移動した。
pub use solvers::original::SweepWorkerStatus;
pub mod state;

pub use state::State;
pub mod value_iterator;

pub use value_iterator::{GridLayers, ValueIterator};
pub mod local;

pub use local::ValueIteratorLocal;
pub mod solvers;

// 収束済み方策から経路 (世界座標の姿勢列) を生成するプランナ層。
// vi_ros2/vi_global_planner (Nav2 の compute_path_to_pose 代替サーバ) の中核。
pub mod planner;

pub use planner::{PathPose, Rollout, RolloutStatus};

// 解けた場を連続に読む制御層 (V̂ 補間 + DWA 型軌道サンプリング)。ソルバの
// bit-exact 検証体制の外側にあり、方策の意味論には触れない。
pub mod ctrl;

// ROS メッセージ「ビュー」と vi_lib 型の変換層 (ROS 非依存)。
// vi_ros2/vi_node と vi_ros2/vi_global_planner が共有する (旧 vi_node/src/bridge.rs)。
pub mod bridge;

// 全地図 belief 推定器 (VIOLA の推定側)。旧 窓つき localize::* の後継。
pub mod belief;

pub use belief::{Belief, BeliefConfig};

// 旧 vi_algorithm から取り込んだ word 並列 bitboard プリミティブ。solvers のフロンティアが
// 使い、vi_bench の bitboard マイクロベンチが `vi_lib::bitboard` として参照する。
pub mod bitboard;
