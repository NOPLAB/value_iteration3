//! vi_planner library — rclrs 非依存モジュール。
//!
//! `core` は vi_reference のみに依存し、ホストの分離クレート方式
//! (CLAUDE.md 参照) で `cargo test --lib` できる。ROS 型との変換・ノード配線は
//! main.rs 側に置く。

pub mod core;
pub mod sink;
