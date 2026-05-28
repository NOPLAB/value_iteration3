//! Constants mirroring `vi_matlab/src/common/vi_params.m` and `fpga/hls/stream/src/vi_stream_types.h`.

use crate::types::{Value, Penalty};

pub const N_ACTIONS: usize = 6;
pub const N_THETA: usize = 60;
pub const MAX_VALUE: Value = 0xFFFF;
pub const PENALTY_OBSTACLE: Penalty = 0xFFFF;
pub const PENALTY_GOAL: Penalty = 0xFFFE;
pub const STEP_COST: u32 = 1;
pub const PROB_BASE: u32 = 262_144;
pub const MAX_OUTCOMES: usize = 10;
pub const TRANS_WORD_STRIDE: usize = 21;
pub const TRANS_TABLE_SIZE: usize = 7_560;

pub const RESOLUTION_XY_BIT: u32 = 6;
pub const RESOLUTION_T_BIT: u32 = 6;

pub const ACTION_FW: [f64; N_ACTIONS] = [0.3, -0.2, 0.0, 0.2, 0.0, 0.2];
pub const ACTION_ROT: [f64; N_ACTIONS] = [0.0, 0.0, -20.0, -20.0, 20.0, 20.0];
