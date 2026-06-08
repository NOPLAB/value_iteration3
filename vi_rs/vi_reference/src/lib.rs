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
pub mod sweep_status;

pub use sweep_status::SweepWorkerStatus;
pub mod state;

pub use state::State;
pub mod value_iterator;

pub use value_iterator::ValueIterator;
