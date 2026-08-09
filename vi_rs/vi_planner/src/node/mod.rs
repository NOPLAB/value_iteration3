//! ノード配線 — **rclrs に依存する側**。`crate::core` (lib 側) が rclrs 非依存の
//! 中核で、こちらは ROS の口だけを持つ。main.rs は起動手順そのものだけを残す。
//!
//! - [`params`] — 起動パラメータの宣言・自己整合検査・sink の置き場の決定。
//! - [`msg`] — ROS メッセージ ⇄ vi_lib の型の変換 (どちらの状態も持たない)。
//! - [`handles`] — 全サーバ・コールバックが共有するハンドル束 ([`handles::Handles`])、
//!   自己位置推定の入れ物 ([`handles::Loc`])、可視化の口。
//! - [`follow_loop`] — 1 ゴールぶんの追従ループ本体 (`core::follow` の 1 tick
//!   判断器とは別物: あちらは行動 1 手、こちらはそれを回す制御ループ)。
//! - [`servers`] — アクションサーバ 4 つと `/goal_pose` 購読の配線。
//! - [`sweep`] — 狭域 → 広域のフィードバック (背景の全域掃きスレッド)。
//! - [`boot`] — /map の待ち受けと、地図 + パラメータからの核の組み立て。

pub mod boot;
pub mod follow_loop;
pub mod handles;
pub mod msg;
pub mod params;
pub mod servers;
pub mod sweep;
