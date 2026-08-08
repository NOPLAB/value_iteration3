//! ベンチの基準値。本家 launch (`value_iteration/launch/*.launch.py`) と同じ
//! 行動集合・姿勢分解能で、`vi_matlab/src/common/vi_params.m` および
//! `vi_fpga/hls/*/src/vi_*_types.h` が持つデータ契約と同じ数値。
//!
//! かつて `vi_core::params` にあり、ROS ノード (`vi_ros2/*`) が「起動パラメータが
//! この値と一致するか」を照合するのに使っていた。ソルバ (`vi_lib`) は行動も
//! θ 数も実行時に受け取る (`ValueIterator::new(actions, threads)` /
//! `cell_num_t`) ので、その照合は launch から値を変える邪魔にしかならず外した。
//! ベンチは「本家と同じ条件で測る」ことが目的なので、値そのものはここに基準として
//! 残す。**ここを変えると過去の測定値と比較できなくなる**ので、掃引したいときは
//! `bench_map` の `--action-scale` のように呼び出し側で倍率を掛けること。
//!
//! HLS / MATLAB 側の同じ値は Rust からは参照していない (それぞれが別に持つ)。

use vi_lib::Action;

/// 本家 launch の行動数。
pub const N_ACTIONS: usize = 6;

/// 姿勢の離散数。本家 `t_resolution_ = 360/cell_num_t_` が整数除算なので、
/// **360 を割り切る値**であること (60 → 6 deg/セル)。
pub const N_THETA: i32 = 60;

/// 行動名。ID 順まで本家 launch と一致する。
pub const ACTION_NAMES: [&str; N_ACTIONS] =
    ["forward", "back", "right", "rightfw", "left", "leftfw"];

/// 各行動の前進量 [m]。
pub const ACTION_FW: [f64; N_ACTIONS] = [0.3, -0.2, 0.0, 0.2, 0.0, 0.2];

/// 各行動の回転量 [deg]。
pub const ACTION_ROT: [f64; N_ACTIONS] = [0.0, 0.0, -20.0, -20.0, 20.0, 20.0];

/// `ACTION_FW` の最大値。セルサイズがこれを超えると 1 手でセル境界を跨げず、
/// 遷移が自セルに潰れて価値が伝播しなくなる (`bench_map` の退化ガード)。
pub const MAX_ACTION_FW_M: f64 = 0.3;

/// 本家 launch と ID 順まで一致する正典 6 行動。
pub fn canonical_actions() -> Vec<Action> {
    (0..N_ACTIONS)
        .map(|i| Action::new(ACTION_NAMES[i], ACTION_FW[i], ACTION_ROT[i], i as i32))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_actions_match_the_reference_launch() {
        let a = canonical_actions();
        assert_eq!(a.len(), N_ACTIONS);
        assert_eq!(a[0].name, "forward");
        assert_eq!((a[0].delta_fw, a[0].delta_rot), (0.3, 0.0));
        assert_eq!(a[1].name, "back");
        assert_eq!((a[1].delta_fw, a[1].delta_rot), (-0.2, 0.0));
        assert_eq!(a[5].name, "leftfw");
        assert_eq!((a[5].delta_fw, a[5].delta_rot), (0.2, 20.0));
        for (i, act) in a.iter().enumerate() {
            assert_eq!(act.id, i as i32);
        }
    }

    #[test]
    fn theta_divides_360() {
        // 割り切れないと t_resolution の整数除算で it が cell_num_t を超える。
        assert_eq!(360 % N_THETA, 0);
    }

    #[test]
    fn max_action_fw_is_the_largest_step() {
        let max = ACTION_FW.iter().cloned().fold(f64::MIN, f64::max);
        assert_eq!(MAX_ACTION_FW_M, max);
    }
}
