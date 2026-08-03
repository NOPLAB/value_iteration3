//! 16bit HLS データ契約のミラー: 型エイリアスのみ。
//!
//! u64 モデルの本体は `vi_reference` が独自に持つ。コスト関数・遷移表・goal マスクの
//! 旧 u16 実装は唯一の利用者だった vi_fixtures とともに撤去した。
//!
//! アルゴリズム定数 (`params`: N_THETA, 行動表, Penalty センチネル) もここから外した。
//! ROS ノード (`vi_ros2/*`) が「起動パラメータがこの定数と一致するか」を照合するのが
//! 唯一の用途だったが、ソルバは行動も θ 数も実行時に受け取る
//! (`ValueIterator::new(actions, threads)` / `cell_num_t`) ので、その照合は launch
//! から値を変える邪魔でしかなかった。ベンチの基準値としての同じ数値は
//! `vi_bench::params` に、HLS / MATLAB 側の契約値は
//! `vi_fpga/hls/*/src/vi_*_types.h` と `vi_matlab/src/common/vi_params.m` にある。
//!
//! この結果 Rust 側でこのクレートを参照しているものは無くなった (残しているのは
//! HLS 契約の型幅の記録として)。
pub mod types;

pub use types::{Value, Penalty, Offset, ThetaIdx, ActionIdx};
