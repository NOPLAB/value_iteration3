//! Test and benchmark fixtures: map generation and transition tables.

pub mod maps;
pub mod transitions;

pub use maps::{GeneratedMap, MapType, generate_map};
pub use transitions::{TransitionMode, generate_transitions};
