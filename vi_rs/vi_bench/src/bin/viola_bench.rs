//! `viola_bench` — VIOLA (Value Iteration with Online Localization and Action)
//! の閉ループ評価。自己位置推定がループに入ると走行がどれだけ劣化するかを測る。
//!
//! 実地図を u64 ソルバで解き (follow_ctrl_bench と同じ規約)、同じ収束場から
//! greedy follow で走行する。ただし世界は **native 解像度**の地図でシミュレート
//! し (真値のレイキャスト・尤度場とも native — vi_planner の実構成と同じ入れ子)、
//! 判断に渡す姿勢だけを 3 way で切り替える:
//!
//! - `truth`  — 真値をそのまま渡す (理想推定の基準線。follow_ctrl_bench の greedy と同条件)。
//! - `dead`   — [`vi_lib::localize::GridLocalizer`] を predict のみで回す
//!   (デッドレコニング。correct の寄与を測るアブレーション)。
//! - `grid`   — predict + observe の**窓つき**ヒストグラム MCL (`GridLocalizer`)。
//! - `belief` — 同じく predict + observe の**全地図** belief ([`vi_lib::belief::Belief`]、
//!   `--viterbi` で min-plus 版)。窓つきとの違いは窓の有無だけではなく、幾何を
//!   スケール後の VI 格子・尤度場を native に取る入れ子まで実ノードと同じになる点
//!   (`grid` は native 一本)。
//!
//! 実行 (v, ω) には毎 tick ガウスノイズが乗る (3 モード共通 — 差は推定だけ)。
//! 推定器へは指令値を渡す (ノイズを知らない = 実機と同じ)。ゴール判定は実ノード
//! と同じく**推定姿勢**で行い停止する — 真値が final でなければ GOAL_BEL
//! (信じて停止) としてそのときの真値誤差を記録する。指標: 到達率 / 所要 tick /
//! 位置誤差 RMS・最大 / 方位誤差 RMS / 観測一致度 / 推定 1 tick の計算時間
//! (40 ms 予算比)。
//!
//! 例 (津田沼 scale 3)。既定ゴール (地図中心) は津田沼では閉じた小領域に落ちて
//! スタートが見つからない (follow_ctrl_bench も同じ) ので、ゴールは明示する:
//! ```text
//! cargo run --release -p vi_bench --bin viola_bench -- \
//!     --starts 6 --goal-x 202.73 --goal-y 27.23
//! ```
//!
//! 誘拐 (kidnapped robot) ベンチ: `--kidnap` で走行中の真値だけを瞬間移動させ、
//! 復帰時間 (t_detect = pose None / t_relock = 誤差 < relock_err_m の連続) と
//! lost 中の計算コストを測る。津田沼では既定センサ (25 m / 36 本) だと開けた
//! 区間で観測が痩せて通常走行すら誤ロックで破綻するので、`--scan-range 60
//! --beam-step 5` が前提 (2026-08-20 計測の構成):
//! ```text
//! cargo run --release -p vi_bench --bin viola_bench -- \
//!     --goal-x 202.73 --goal-y 27.23 \
//!     --start-x 150 --start-y 33 --start-theta-deg 0 \
//!     --kidnap "label=K1,t=30,x=100,y=35,yaw=90" \
//!     --modes grid,adaptive,belief --trials 3 --max-ticks 9000 \
//!     --scan-range 60 --beam-step 5
//! ```
//! 既知の結果 (2026-08-20): 静止した受動復帰は全滅する — 廊下は並進対称で
//! 原理的に判別不能 (ESS ~900 で凍結)、特徴的な交差点でも尤度飽和の同値セル塊
//! (~236) が contract_ess (既定 50) に届かず、contract を塊より上げると多峰の
//! まま解放して誤姿勢を返す。
//!
//! 能動的再定位 (`--active-reloc`、本家 follow_loop のロスト分岐の写像 —
//! 判別場は主フィールドを in place で張り替え、re-lock 後に解き直す) も
//! 同日計測で出荷状態では機能しない。原因 3 段: (1) 判別変位の幾何が屋内定数
//! で屋外は全候補スコア 0 → `reloc_scale` (BeliefConfig へ昇格、津田沼 4.0)
//! で解消; (2) QMDP が生 top-64 セル評価で veto ロック (NoAction 94%) —
//! `vi_lib::belief::weighted_modes` への集約で belief は解消 (この bench の
//! reloc 分岐は集約が既定。実ノード follow_loop は未配線)、adaptive は
//! 粗レベル仮説の非 free 写像で残る; (3) belief のリセット過渡の一時集中で
//! ESS 解除が早発し誤姿勢を返す → 解除条件に単峰ゲート (`mode_count` ≤ 1)
//! を追加して解消。K1 廊下は scale 4 でも targets 空 — 判別点 (交差点) は
//! 数十 m 先で δ≤12 m の局所探索では原理的に届かない。
//!
//! 判別機動の実行も 2 つの実行器の失敗 (QMDP: 多目標場の最寄り割当が食い違い
//! 回頭拮抗で 0.3 m / naive open-loop δ*: 真値が top-4 モード外だと近接ガードと
//! デッドロック) を経て `ctrl::lost_creep` (`--reloc-creep`) + 相関観測ゲート
//! (`--lost-min-d 0.2 --lost-min-a-deg 30`、AMCL update_min_d/a 相当) で解決 —
//! ロスト中の安全な判別走行 6〜25 m を達成 (再現には `--reloc-timeout-s 300`
//! も必要 — 既定 30 s は誤解除前に打ち切って LOST 停止で終わる。sim は決定論的
//! で、この構成なら全指標 bit 再現する)。それでも K1〜K4 全て最後は**遠方
//! エイリアスへの誤解除** (err 170〜238 m、quality 0.5〜0.8) で終わる: 60 m
//! センサでも「幅 W の廊下」の署名はキャンパス内の複数箇所と 30 m 走っても
//! 一致し続け、崩壊の勝者はほぼコイントス (証拠を濃くする実験は誤収束を
//! 速めただけ)。終端の壁は尤度場の場所弁別力 — 次手は解除の probation
//! (解除直後を仮 re-lock とし、短い検証走行の予測整合で確定) か、より豊かな
//! 場所署名。

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use clap::Parser;

use vi_bench::params::N_THETA;
use vi_bench::pgm;
use vi_bench::sim::{
    chamfer_dist, greedy_decide, in_field, mean, scaled_actions, snap_to_free, GreedyOut, Rng,
};
use vi_lib::bridge::PoseView;
use vi_lib::ctrl::{unicycle_step, CostView};
use vi_lib::belief::{Belief, BeliefConfig as WholeBeliefConfig};
use vi_lib::localize::{cast_scan, AdaptiveLocalizer, BeliefConfig, GridLocalizer, Localizer};
use vi_lib::params::MAX_COST;
use vi_lib::planner::{pose_to_cell, qmdp_decide, PolicyView, QmdpDecision};
use vi_lib::solvers::{solve, U64Solver};
use vi_lib::{OccupancyGrid, Quaternion, ValueIterator};

#[derive(Parser)]
#[command(about = "Closed-loop VIOLA benchmark: greedy follow with localization in the loop.")]
struct Args {
    /// Map YAML (bench_map と同じ規約)。既定は同梱の津田沼キャンパス地図。
    #[arg(long)]
    map: Option<PathBuf>,

    /// VI 側の整数ダウンサンプル係数 (密な states を持つので RAM に注意 —
    /// 津田沼 scale 3 で約 12.6 GB)。尤度場・レイキャストは常に native (scale 1)。
    /// scale 6 は粗視化で中心ゴール域が閉塞して不成立 (follow_ctrl_bench と同じ)。
    #[arg(long, default_value_t = 3)]
    scale: usize,

    /// ゴール X/Y [m] (省略時は地図中心へスナップ) と方位 [deg]。
    #[arg(long)]
    goal_x: Option<f64>,
    #[arg(long)]
    goal_y: Option<f64>,
    #[arg(long, default_value_t = 90.0)]
    goal_theta_deg: f64,
    #[arg(long)]
    goal_radius_m: Option<f64>,
    #[arg(long, default_value_t = 15)]
    goal_margin_theta_deg: i32,

    /// unknown (グレー) セルの扱い。
    #[arg(long, value_enum, default_value_t = UnknownMode::Obstacle)]
    unknown: UnknownMode,

    /// 安全膨張半径 [m] / 膨張域ペナルティ。
    #[arg(long, default_value_t = 0.6)]
    safety_radius_m: f64,
    #[arg(long, default_value_t = 100000.0)]
    safety_penalty: f64,

    /// 前進歩幅の倍率 (bench_map と同じ意味)。前進歩幅が 2 セルを切ると
    /// 値伝播が退化する (scale 3 の 0.3 m はちょうど境界で可)。
    #[arg(long, default_value_t = 1.0)]
    action_scale: f64,

    /// ソルバ名 (`U64Solver::from_name`)。
    #[arg(long, default_value = "frontier2d_sparse")]
    solver: String,
    #[arg(long, default_value_t = 10_000_000)]
    max_iters: u32,

    /// スタート地点の数 / 乱択シード / ゴール距離範囲 [m]。
    #[arg(long, default_value_t = 6)]
    starts: usize,
    #[arg(long, default_value_t = 42)]
    seed: u64,
    #[arg(long, default_value_t = 20.0)]
    min_start_m: f64,
    #[arg(long, default_value_t = 60.0)]
    max_start_m: f64,

    /// 固定スタート [m]/[deg] (両方指定で乱択の代わりにこの 1 点を `--trials` 回)。
    #[arg(long)]
    start_x: Option<f64>,
    #[arg(long)]
    start_y: Option<f64>,
    #[arg(long, default_value_t = 0.0)]
    start_theta_deg: f64,
    /// 固定スタート時の試行数 (ノイズ・シードずらしだけ変えて繰り返す)。
    #[arg(long, default_value_t = 1)]
    trials: usize,

    /// 誘拐シナリオ `label=K1,t=30,x=100,y=35,yaw=90` (繰り返し可、yaw は絶対 [deg])。
    /// 指定すると各シナリオに加えて誘拐なしの `base` も走る。
    #[arg(long = "kidnap")]
    kidnap: Vec<String>,

    /// 走らせるモード (カンマ区切り: truth,dead,grid,adaptive,belief)。
    #[arg(long, default_value = "truth,dead,grid,belief")]
    modes: String,

    /// 復帰 (re-lock) 判定: 推定誤差しきい値 [m] とその連続 tick 数。
    #[arg(long, default_value_t = 1.0)]
    relock_err_m: f64,
    #[arg(long, default_value_t = 20)]
    relock_hold_ticks: usize,

    /// 制御周期 [s] / 1 走行の tick 上限。
    #[arg(long, default_value_t = 0.1)]
    tick_s: f64,
    #[arg(long, default_value_t = 6000)]
    max_ticks: usize,

    /// greedy の近傍借用半径 (チェビシェフ、セル)。
    #[arg(long, default_value_t = 4)]
    action_tolerance_cells: i32,

    /// 実行ノイズ: 毎 tick の v [m/s] / ω [deg/s] に乗るガウス σ。
    #[arg(long, default_value_t = 0.02)]
    noise_v: f64,
    #[arg(long, default_value_t = 2.0)]
    noise_w_deg: f64,

    /// シードの真値からのずらし量 [m] / [deg] (方向は乱択)。
    #[arg(long, default_value_t = 0.1)]
    seed_offset_m: f64,
    #[arg(long, default_value_t = 5.0)]
    seed_offset_deg: f64,

    /// スキャンのビーム数と最大レンジ [m] (belief の max_range も兼ねる)。
    #[arg(long, default_value_t = 360)]
    scan_beams: usize,
    #[arg(long, default_value_t = 25.0)]
    scan_range: f64,

    /// 推定器のチューニング ([`BeliefConfig`] 対応、実機の校正ノブと同じ)。
    /// `belief_half_m` は窓つき (`grid`) だけに効く。
    #[arg(long, default_value_t = 2.5)]
    belief_half_m: f64,
    #[arg(long, default_value_t = 0.2)]
    belief_sigma_m: f64,
    #[arg(long, default_value_t = 10)]
    beam_step: usize,
    #[arg(long, default_value_t = 0.03)]
    motion_sigma_xy: f64,
    #[arg(long, default_value_t = 2.0)]
    motion_sigma_theta_deg: f64,
    #[arg(long, default_value_t = 0.3)]
    init_sigma_xy: f64,
    #[arg(long, default_value_t = 10.0)]
    init_sigma_theta_deg: f64,

    /// 全地図 belief (`belief` モード) を min-plus (MAP / Viterbi) 更新則で回す。
    #[arg(long)]
    viterbi: bool,

    /// 全地図 belief のロスト進入 / 解除 ESS。**絶対セル数**なので地図スケール
    /// 依存 — 既定 (500/50) は TB3 級。津田沼 0.15 m 格子では健全な追跡の ESS
    /// が進入しきい値を超えてラッチが永久に降りなくなるため、両方を上げる
    /// (進入/解除 10 倍ヒステリシスを保つ)。
    #[arg(long, default_value_t = 500.0)]
    lost_ess: f64,
    #[arg(long, default_value_t = 50.0)]
    contract_ess: f64,

    /// ロスト中に判別点の多目標場を解いて QMDP で走る能動的再定位
    /// (実ノードの `active_reloc` 相当)。reloc_targets を持つ adaptive / belief
    /// でだけ意味がある。判別場の solve は sim 時間 0 で完了し、実時間コストは
    /// reloc_solve_s として別掲する (実ノードでは solve 中ロボットは停止)。
    #[arg(long)]
    active_reloc: bool,
    /// 能動的再定位を諦めるまでの時間 [s] (本家 RELOC_TIMEOUT_SEC = 30)。
    #[arg(long, default_value_t = 30.0)]
    reloc_timeout_s: f64,
    /// 判別変位探索の幾何スケール (`BeliefConfig::reloc_scale`)。1.0 = 屋内基準。
    /// 屋外道路 (クリアランス数 m) では署名リングが尤度場に届かずスコア 0 に
    /// 潰れるので上げる (津田沼は 4.0 目安)。
    #[arg(long, default_value_t = 1.0)]
    reloc_scale: f64,
    /// 判別機動を [`vi_lib::ctrl::lost_creep`] (スキャン反射の安全並進、δ* は
    /// 方位バイアス) で実行する。多目標 VI + QMDP (既定) は最寄り割当の食い違い
    /// で回頭が拮抗し、収束レースに負ける — creep は判別場の solve も不要。
    #[arg(long)]
    reloc_creep: bool,
    /// ロスト中の相関観測ゲート (`BeliefConfig::lost_update_min_d_m` /
    /// `lost_update_min_a_deg` — AMCL の update_min_d/a 相当)。0 = 従来挙動。
    /// 判別機動と併用するなら 0.2 / 30 が目安。
    #[arg(long, default_value_t = 0.0)]
    lost_min_d: f64,
    #[arg(long, default_value_t = 0.0)]
    lost_min_a_deg: f64,

    /// CSV 出力先 (省略時は標準出力の表のみ)。
    #[arg(long)]
    out: Option<PathBuf>,

    /// 軌跡 CSV の出力ディレクトリ (run ごとに traj_<spec>_<mode>_<start>.csv)。
    #[arg(long)]
    traj_dir: Option<PathBuf>,
}

#[derive(Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum UnknownMode {
    Obstacle,
    Free,
}

fn default_map_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/map_tsudanuma.yaml")
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Truth,
    Dead,
    Grid,
    Adaptive,
    Whole,
}
impl Mode {
    fn name(self) -> &'static str {
        match self {
            Mode::Truth => "truth",
            Mode::Dead => "dead",
            Mode::Grid => "grid",
            Mode::Adaptive => "adaptive",
            Mode::Whole => "belief",
        }
    }
    fn from_name(s: &str) -> Option<Mode> {
        Some(match s {
            "truth" => Mode::Truth,
            "dead" => Mode::Dead,
            "grid" => Mode::Grid,
            "adaptive" => Mode::Adaptive,
            "belief" => Mode::Whole,
            _ => return None,
        })
    }
}

/// 誘拐シナリオ: `at_tick` で真値だけを瞬間移動させる (推定器は知らない)。
#[derive(Clone, Copy)]
struct Kidnap {
    at_tick: usize,
    x: f64,
    y: f64,
    yaw_rad: f64,
}

/// `label=K1,t=30,x=100,y=35,yaw=90` を (label, Kidnap) に。yaw は絶対 [deg]。
fn parse_kidnap(s: &str, tick_s: f64) -> Result<(String, Kidnap), String> {
    let (mut label, mut t, mut x, mut y, mut yaw) = (None, None, None, None, 90.0f64);
    for part in s.split(',') {
        let (k, v) = part.split_once('=').ok_or_else(|| format!("bad part: {part}"))?;
        let v = v.trim();
        match k.trim() {
            "label" => label = Some(v.to_string()),
            "t" => t = v.parse::<f64>().ok(),
            "x" => x = v.parse::<f64>().ok(),
            "y" => y = v.parse::<f64>().ok(),
            "yaw" => yaw = v.parse::<f64>().map_err(|e| e.to_string())?,
            other => return Err(format!("unknown key: {other}")),
        }
    }
    let (Some(t), Some(x), Some(y)) = (t, x, y) else {
        return Err("t=, x=, y= are required".into());
    };
    Ok((
        label.unwrap_or_else(|| "kidnap".into()),
        Kidnap { at_tick: (t / tick_s).round() as usize, x, y, yaw_rad: yaw.to_radians() },
    ))
}

/// 推定器の入れ物。窓つき ([`GridLocalizer`], `Localizer` トレイト) と全地図
/// ([`Belief`]) は別の型なので、bench の中だけの都合でここに束ねる
/// (実ノード側の同じ役の enum は `vi_planner::node::handles::Loc`)。
enum Est {
    /// 窓つき系 ([`GridLocalizer`] / [`AdaptiveLocalizer`]、`Localizer` トレイト)。
    Win(Box<dyn Localizer>),
    Whole(Box<Belief>),
}

impl Est {
    fn predict(&mut self, v: f64, w_deg: f64, dt: f64) {
        match self {
            Est::Win(l) => l.predict(v, w_deg, dt),
            Est::Whole(b) => b.predict(v, w_deg, dt),
        }
    }
    fn observe(&mut self, scan: &vi_lib::msg::LaserScan) {
        match self {
            Est::Win(l) => l.observe(scan),
            Est::Whole(b) => b.observe(scan),
        }
    }
    fn pose(&self) -> Option<PoseView> {
        match self {
            Est::Win(l) => l.pose(),
            Est::Whole(b) => b.pose(),
        }
    }
    fn quality(&self) -> f64 {
        match self {
            Est::Win(l) => l.quality(),
            Est::Whole(b) => b.quality(),
        }
    }
    /// belief の広がり (全地図のみ — 窓つきはトレイトが出していない)。
    fn ess(&self) -> f64 {
        match self {
            Est::Win(_) => f64::NAN,
            Est::Whole(b) => b.ess(),
        }
    }
    fn top_cells(&self, k: usize) -> Vec<(PoseView, f64)> {
        match self {
            Est::Win(l) => l.top_cells(k),
            Est::Whole(b) => b.top_cells(k),
        }
    }
    fn reloc_targets(&self) -> Vec<(f64, f64)> {
        match self {
            Est::Win(l) => l.reloc_targets(),
            Est::Whole(b) => b.reloc_targets(),
        }
    }
}

#[derive(Clone)]
struct RunResult {
    mode: &'static str,
    reached: bool,
    /// 推定上のゴール到達で停止した (実ノードの成功宣言と同じ)。truth が
    /// final でなければ「信じて止まった」— そのときの真値誤差が `final_err_m`。
    believed_goal: bool,
    final_err_m: f64,
    collided: bool,
    out_of_map: bool,
    starved: bool,
    ticks: usize,
    path_len_m: f64,
    err_sq_sum: f64,
    err_max_m: f64,
    yaw_sq_sum: f64,
    err_samples: u64,
    quality_sum: f64,
    loc_us_sum: f64,
    loc_us_max: f64,
    /// 走行中の壁とのクリアランス (真値セル → 最寄り障害物の L∞ 距離 [m])。
    /// マージン膨張 (`sigma_margin_gain`) が効いているかはここに出る — 上田ら 2023
    /// が「隘路 / 大回り」の 2 値で数えたものの連続版で、衝突はいつも最小値の側で
    /// 起きる。
    clear_min_m: f64,
    clear_sum_m: f64,
    clear_samples: u64,
    /// 誘拐シナリオの復帰計測 (kidnap 指定時のみ有効)。tick は誘拐からの相対。
    kidnapped: bool,
    /// lost 宣言 (pose() が None になった最初の tick)。grid はロスト状態を
    /// 持たないので None のまま。
    detect_tick: Option<usize>,
    /// 復帰: 推定誤差 < relock_err_m が relock_hold_ticks 連続した最初の tick。
    relock_tick: Option<usize>,
    /// lost (pose None) が連続しすぎて打ち切った (静止スキャンだけでは判別
    /// できないケース — 実ノードなら active_reloc の出番)。
    lost_gave_up: bool,
    /// 誘拐 → 復帰確定までの窓の推定計算時間 (復帰コスト)。
    lost_us_sum: f64,
    lost_us_max: f64,
    lost_us_ticks: u64,
    /// 能動的再定位 (--active-reloc): 判別場の構築+solve の実時間合計 [s]
    /// (sim 時間には入れない — 実ノードでは solve 中ロボットは停止)、solve
    /// 回数、ロスト中に QMDP で走った距離 [m]。
    reloc_solve_s: f64,
    reloc_solves: u32,
    reloc_dist_m: f64,
}

impl RunResult {
    fn err_rms_m(&self) -> f64 {
        (self.err_sq_sum / self.err_samples.max(1) as f64).sqrt()
    }
    fn clear_mean_m(&self) -> f64 {
        if self.clear_samples == 0 {
            return f64::NAN;
        }
        self.clear_sum_m / self.clear_samples as f64
    }
    fn yaw_rms_deg(&self) -> f64 {
        (self.yaw_sq_sum / self.err_samples.max(1) as f64).sqrt().to_degrees()
    }
    fn quality_mean(&self) -> f64 {
        if self.err_samples == 0 {
            return f64::NAN;
        }
        self.quality_sum / self.err_samples as f64
    }
}

fn wrap_rad(d: f64) -> f64 {
    use std::f64::consts::PI;
    (d + PI).rem_euclid(2.0 * PI) - PI
}


#[allow(clippy::too_many_arguments)]
fn simulate(
    // 能動的再定位が判別場を **in place** で張り替える (実ノードのキャッシュ
    // 差し替えと同じ「一度に 1 枚」規律 — 第 2 フィールドは RAM に載らない)
    // ため &mut。run の出口で必ず主ゴールの場に戻す。
    vi: &mut ValueIterator,
    grid: &OccupancyGrid,
    native: &OccupancyGrid,
    // 障害物までの距離場 (chamfer_dist、3 = 1 セル)。grid と同じ格子。
    chamfer: &[u32],
    mode: Mode,
    start: (f64, f64, f64),
    goal: (f64, f64),
    seed_pose: PoseView,
    bc: BeliefConfig,
    wbc: WholeBeliefConfig,
    noise_seed: u64,
    kidnap: Option<Kidnap>,
    solver: U64Solver,
    goal_radius: f64,
    traj: Option<&mut String>,
    args: &Args,
) -> RunResult {
    let mut traj = traj;
    let (mut x, mut y, mut yaw) = start;
    let mut rng = Rng(noise_seed.max(1));
    let mut loc = (mode != Mode::Truth).then(|| match mode {
        Mode::Whole => {
            // 幾何 (belief 格子) は VI と同じスケール後の grid、尤度場だけ native。
            // 実ノードと同じ入れ子 — 窓つきの GridLocalizer は native 一本。
            let mut b = Belief::new(grid, N_THETA, native, wbc);
            b.seed(seed_pose);
            Est::Whole(Box::new(b))
        }
        Mode::Adaptive => {
            let mut l = AdaptiveLocalizer::new(native, N_THETA, bc);
            l.set_pose(seed_pose);
            Est::Win(Box::new(l))
        }
        _ => {
            let mut l = GridLocalizer::new(native, N_THETA, bc);
            l.set_pose(seed_pose);
            Est::Win(Box::new(l))
        }
    });
    let mut r = RunResult {
        mode: mode.name(),
        reached: false,
        believed_goal: false,
        final_err_m: 0.0,
        collided: false,
        out_of_map: false,
        starved: false,
        ticks: 0,
        path_len_m: 0.0,
        err_sq_sum: 0.0,
        err_max_m: 0.0,
        yaw_sq_sum: 0.0,
        err_samples: 0,
        quality_sum: 0.0,
        loc_us_sum: 0.0,
        loc_us_max: 0.0,
        clear_min_m: f64::INFINITY,
        clear_sum_m: 0.0,
        clear_samples: 0,
        kidnapped: kidnap.is_some(),
        detect_tick: None,
        relock_tick: None,
        lost_gave_up: false,
        lost_us_sum: 0.0,
        lost_us_max: 0.0,
        lost_us_ticks: 0,
        reloc_solve_s: 0.0,
        reloc_solves: 0,
        reloc_dist_m: 0.0,
    };
    let mut starve = 0usize;
    // 能動的再定位 (実ノードの Reloc::Idle/Driving/GaveUp に対応)。判別場は
    // ロスト 1 回につき 1 度だけ、主フィールドを in place で張り替えて解く。
    let mut field_is_reloc = false;
    let mut reloc_ticks = 0u32;
    let mut reloc_gave_up = false;
    // --reloc-creep: ロスト 1 回ぶんの凍結判別点 (pose 復帰で捨てる)。
    let mut reloc_frozen: Option<Vec<(f64, f64)>> = None;
    // δ* の実行状態 (残距離 [m], ロボット系方位 [rad])。凍結時に上位モードから
    // 1 度だけ落とし、以後は指令オドメトリで更新する — 上位モードの同一性は
    // エイリアス間で毎 tick 入れ替わるので、モードから引き直すと方位が振動して
    // 回頭に食われる (実測: 180 s で並進 2.5 m)。
    let mut reloc_delta: Option<(f64, f64)> = None;
    // QMDP 決定の内訳 (診断用: 判別場が立ったのに走らないときの切り分け)。
    let (mut qn_goal, mut qn_act, mut qn_noact) = (0u32, 0u32, 0u32);
    let reloc_ticks_limit = (args.reloc_timeout_s / args.tick_s).ceil() as u32;
    /// 本家 follow_loop の近接ガード [m] (姿勢が無く local_penalty を置けない間の停止距離)。
    const RELOC_STOP_RANGE: f64 = 0.35;
    /// 本家 follow_loop の QMDP 仮説上限。
    const QMDP_TOP_K: usize = 64;
    // 復帰判定のストリーク (連続 tick 数と開始 tick) / lost 連続カウンタ。
    let mut relock_streak = 0usize;
    let mut relock_streak_from = 0usize;
    let mut lost_run = 0usize;
    // 静止スキャンだけで判別できないときの打ち切り (60 s)。実ノードなら
    // active_reloc が動く領分で、この bench では受動復帰だけを測る。
    let lost_abort_ticks = (60.0 / args.tick_s) as usize;

    for t in 0..args.max_ticks {
        // 誘拐: 真値だけを瞬間移動 (推定器は predict/observe の系列しか知らない)。
        if let Some(k) = kidnap {
            if t == k.at_tick {
                (x, y, yaw) = (k.x, k.y, k.yaw_rad);
            }
        }
        let after_kidnap = kidnap.map(|k| t >= k.at_tick).unwrap_or(false);
        let recovering = after_kidnap && r.relock_tick.is_none();
        // 物理イベント (地図外・衝突・ゴール到達) は真値で判定する。
        let (tix, tiy, tit) = pose_to_cell(vi, x, y, yaw);
        if !in_field(vi, tix, tiy, tit) {
            r.out_of_map = true;
            break;
        }
        if !CostView::free_at(vi, tix, tiy) {
            r.collided = true;
            break;
        }
        if !field_is_reloc && vi.is_final(tix, tiy, tit) {
            // 判別場の final は再定位の行き先であってゴールではない。
            r.reached = true;
            break;
        }
        let mut clr_now = f64::INFINITY;
        if let Some(&d) = chamfer.get((tiy * grid.width + tix) as usize) {
            let c = d as f64 / 3.0 * grid.resolution; // chamfer は 3-4 重み (3 = 1 セル)
            clr_now = c;
            r.clear_min_m = r.clear_min_m.min(c);
            r.clear_sum_m += c;
            r.clear_samples += 1;
        }

        // 判断は推定姿勢で引く (truth モードは真値がそのまま推定)。
        let est = match &loc {
            None => Some(PoseView { x, y, yaw_rad: yaw }),
            Some(l) => l.pose(),
        };
        if let (Some(e), Some(_)) = (est, &loc) {
            let d = ((e.x - x).powi(2) + (e.y - y).powi(2)).sqrt();
            r.err_sq_sum += d * d;
            r.err_max_m = r.err_max_m.max(d);
            r.yaw_sq_sum += wrap_rad(e.yaw_rad - yaw).powi(2);
            r.err_samples += 1;
            r.quality_sum += loc.as_ref().map(|l| l.quality()).unwrap_or(1.0);
        }

        if let Some(buf) = traj.as_deref_mut() {
            use std::fmt::Write;
            let (ex, ey, eyaw) = est
                .map(|e| (e.x, e.y, e.yaw_rad))
                .unwrap_or((f64::NAN, f64::NAN, f64::NAN));
            let _ = writeln!(
                buf,
                "{t},{x:.3},{y:.3},{:.4},{ex:.3},{ey:.3},{eyaw:.4},{:.2},{:.0}",
                yaw,
                loc.as_ref().map(|l| l.quality()).unwrap_or(1.0),
                loc.as_ref().map(|l| l.ess()).unwrap_or(f64::NAN),
            );
        }

        // 誘拐後の復帰計測。detect = pose None (lost 宣言)、relock = 推定誤差 <
        // relock_err_m が relock_hold_ticks 連続 (その最初の tick を数える)。
        if after_kidnap && loc.is_some() {
            let at = kidnap.unwrap().at_tick;
            if est.is_none() {
                if r.detect_tick.is_none() {
                    r.detect_tick = Some(t - at);
                }
                // 能動的再定位が生きている間は「静止のまま判別不能」に数えない
                // (走って判別しに行っている最中なので)。
                let reloc_live =
                    args.active_reloc && !reloc_gave_up && reloc_ticks < reloc_ticks_limit;
                if !reloc_live {
                    lost_run += 1;
                    if lost_run > lost_abort_ticks {
                        r.lost_gave_up = true;
                        r.ticks += 1;
                        break;
                    }
                }
            } else {
                lost_run = 0;
            }
            if r.relock_tick.is_none() {
                let locked = est
                    .map(|e| ((e.x - x).powi(2) + (e.y - y).powi(2)).sqrt() < args.relock_err_m)
                    .unwrap_or(false);
                if locked {
                    if relock_streak == 0 {
                        relock_streak_from = t;
                    }
                    relock_streak += 1;
                    if relock_streak >= args.relock_hold_ticks {
                        r.relock_tick = Some(relock_streak_from - at);
                    }
                } else {
                    relock_streak = 0;
                }
            }
        }

        // 推定姿勢で方策を引く。推定上のゴールなら実ノードと同じく停止して
        // 成功を宣言する — 真値が final でなければ「信じて止まった」となり、
        // そのときの真値のゴール中心距離を final_err_m に残す。
        let mut cmd: Option<(f64, f64)> = None;
        let mut from_reloc = false;
        if let Some(e) = est {
            // pose が戻った — 本来のゴールを解き直して通常追従へ (本家の再ロック後
            // re-solve と Reloc::Idle 復帰)。
            if field_is_reloc {
                let t0 = Instant::now();
                vi.set_goal(goal.0, goal.1, args.goal_theta_deg as i32);
                solve(vi, solver, args.max_iters);
                r.reloc_solve_s += t0.elapsed().as_secs_f64();
                r.reloc_solves += 1;
                field_is_reloc = false;
            }
            reloc_ticks = 0;
            reloc_gave_up = false;
            reloc_frozen = None;
            reloc_delta = None;
            let (ix, iy, it) = pose_to_cell(vi, e.x, e.y, e.yaw_rad);
            if in_field(vi, ix, iy, it) {
                match greedy_decide(vi, ix, iy, it, args.action_tolerance_cells) {
                    GreedyOut::Goal => {
                        r.believed_goal = true;
                        r.final_err_m = ((x - goal.0).powi(2) + (y - goal.1).powi(2)).sqrt();
                        break;
                    }
                    GreedyOut::Act(fw, rot) => cmd = Some((fw, rot)),
                    GreedyOut::NoAction => {}
                }
            }
        } else if args.active_reloc && !reloc_gave_up && reloc_ticks < reloc_ticks_limit {
            // ロスト中の能動的再定位 (本家 follow_loop のロスト分岐を写す):
            // 仮説 2 個以上 + 近接ガードのとき、初回は判別点の多目標場を解き、
            // 以後は QMDP で判別点へ走る。Goal/NoAction は止まって観測を待つ。
            if let Some(l) = &loc {
                reloc_ticks += 1;
                // QMDP へは生の top-64 でなくモード集約を渡す (vi_lib 側の
                // (b) 修正 — 生セルはキャンパス級の多峰で veto ロックする)。
                let hyps = vi_lib::belief::weighted_modes(&l.top_cells(QMDP_TOP_K), 1.0, 4);
                // 近接ガードは chamfer クリアランスで代用 (本家は生スキャン最近接)。
                if args.reloc_creep {
                    // creep 実行器: 安全判定はスキャン (真値の環境) のみ —
                    // 仮説地図の近接ガード (clr_now) は使わない。δ* があれば
                    // 上位モード系の最寄り判別点への方位をバイアスに渡す。
                    if reloc_frozen.is_none() {
                        let targets = l.reloc_targets();
                        if !targets.is_empty() {
                            // δ* を (残距離, ロボット系方位) へ 1 度だけ落とす。
                            if let Some(&(mp, _)) = hyps.first() {
                                if let Some((tx, ty)) = targets
                                    .iter()
                                    .copied()
                                    .min_by(|a, b| {
                                        (a.0 - mp.x)
                                            .hypot(a.1 - mp.y)
                                            .total_cmp(&(b.0 - mp.x).hypot(b.1 - mp.y))
                                    })
                                {
                                    reloc_delta = Some((
                                        (tx - mp.x).hypot(ty - mp.y),
                                        wrap_rad((ty - mp.y).atan2(tx - mp.x) - mp.yaw_rad),
                                    ));
                                }
                            }
                            reloc_frozen = Some(targets);
                        }
                    }
                    let preferred = reloc_delta.map(|(_, b)| b);
                    let scan = cast_scan(
                        native,
                        PoseView { x, y, yaw_rad: yaw },
                        args.scan_beams,
                        args.scan_range,
                    );
                    let (fw, rot) = vi_lib::ctrl::lost_creep(&scan, preferred, args.scan_range);
                    if reloc_ticks % 25 == 1 {
                        eprintln!(
                            "  [creep] t={t} frozen={} pref_deg={:?} cmd=({fw:.2},{rot:.0}) yaw={:.0}",
                            reloc_frozen.is_some(),
                            preferred.map(|b| b.to_degrees() as i32),
                            yaw.to_degrees().rem_euclid(360.0),
                        );
                    }
                    // 診断: qn_act = 前進 tick / qn_goal = 回頭 tick / qn_noact = 停止。
                    if fw != 0.0 {
                        qn_act += 1;
                    } else if rot != 0.0 {
                        qn_goal += 1;
                    } else {
                        qn_noact += 1;
                    }
                    if fw != 0.0 || rot != 0.0 {
                        cmd = Some((fw, rot));
                        from_reloc = true;
                    }
                } else if hyps.len() >= 2 && clr_now >= RELOC_STOP_RANGE {
                    if !field_is_reloc {
                        let targets = l.reloc_targets();
                        if reloc_ticks % 50 == 1 {
                            // 判別場が立たない理由の診断 (モード数は top-64 の近似)。
                            eprintln!(
                                "  [reloc] t={t} hyps={} modes={} targets={} ess={:.0}",
                                hyps.len(),
                                vi_lib::belief::mode_count(&hyps, 1.0),
                                targets.len(),
                                l.ess(),
                            );
                        }
                        if !targets.is_empty() {
                            // 主フィールドを判別場に張り替える (実ノードのキャッシュ
                            // 差し替え相当 — 第 2 フィールドは確保しない)。
                            let t0 = Instant::now();
                            vi.set_goal_region(&targets, goal_radius.max(grid.resolution));
                            let st = solve(vi, solver, args.max_iters);
                            r.reloc_solve_s += t0.elapsed().as_secs_f64();
                            r.reloc_solves += 1;
                            field_is_reloc = true;
                            if !st.converged {
                                // 収束しない判別場では走らない — 主ゴールへ戻して諦める。
                                vi.set_goal(goal.0, goal.1, args.goal_theta_deg as i32);
                                solve(vi, solver, args.max_iters);
                                field_is_reloc = false;
                                reloc_gave_up = true;
                            }
                        }
                    }
                    if field_is_reloc {
                        match qmdp_decide(vi, &hyps) {
                            QmdpDecision::Action(i) => {
                                qn_act += 1;
                                let (fw, rot) = vi.action_delta(i);
                                cmd = Some((fw, rot));
                                from_reloc = true;
                            }
                            // Goal = 判別点に到着 / NoAction — 止まって観測を待つ。
                            QmdpDecision::Goal => qn_goal += 1,
                            QmdpDecision::NoAction => qn_noact += 1,
                        }
                    }
                }
            }
        }

        // cmd が引けない tick はロボットを止める (実機の no-action と同じ) が、
        // スキャンの correct は実ノード同様に止まらない — observe は毎 tick 呼ぶ
        // (predict は動いた tick だけ、本家の呼び出し規約)。lost 中 (pose None)
        // の停止は policy 飢餓に数えない — それは復帰待ちであって故障ではない。
        let moved = if let Some((v, w_deg)) = cmd {
            starve = 0;
            // 実行 (真値側) にはノイズが乗り、推定器は指令値しか知らない。
            let v_exec = v + args.noise_v * rng.gauss();
            let w_exec = w_deg + args.noise_w_deg * rng.gauss();
            let (nx, ny, nyaw) =
                unicycle_step(x, y, yaw, v_exec, w_exec.to_radians(), args.tick_s);
            let d = ((nx - x).powi(2) + (ny - y).powi(2)).sqrt();
            r.path_len_m += d;
            if from_reloc {
                r.reloc_dist_m += d;
            }
            (x, y, yaw) = (nx, ny, nyaw);
            Some((v, w_deg))
        } else {
            if est.is_some() {
                starve += 1;
                if starve > 50 {
                    r.starved = true;
                    r.ticks += 1;
                    break;
                }
            }
            None
        };

        // δ* 実行状態の指令オドメトリ更新 (ロボット系の目標点を平行移動+回転)。
        if let (Some((d, b)), Some((v, w_deg))) = (&mut reloc_delta, moved) {
            let (bx, by) = (*d * b.cos() - v * args.tick_s, *d * b.sin());
            *b = wrap_rad(by.atan2(bx) - (w_deg.to_radians()) * args.tick_s);
            *d = bx.hypot(by);
            if *d < 0.5 {
                reloc_delta = None; // 判別点に到着 — 以後は開けた方向へ這う
            }
        }

        if let Some(l) = &mut loc {
            let t0 = Instant::now();
            if let Some((v, w_deg)) = moved {
                l.predict(v, w_deg, args.tick_s);
            }
            if mode != Mode::Dead {
                let scan = cast_scan(
                    native,
                    PoseView { x, y, yaw_rad: yaw },
                    args.scan_beams,
                    args.scan_range,
                );
                l.observe(&scan);
            }
            let us = t0.elapsed().as_secs_f64() * 1e6;
            r.loc_us_sum += us;
            r.loc_us_max = r.loc_us_max.max(us);
            if recovering {
                r.lost_us_sum += us;
                r.lost_us_max = r.lost_us_max.max(us);
                r.lost_us_ticks += 1;
            }
        }
        r.ticks += 1;
    }
    // 判別場のまま終わった run (LOST 等) は主ゴールへ戻す — 次の run が同じ
    // フィールドを共有するための不変条件。
    if field_is_reloc {
        vi.set_goal(goal.0, goal.1, args.goal_theta_deg as i32);
        solve(vi, solver, args.max_iters);
    }
    if r.reloc_solves > 0 || qn_act + qn_goal + qn_noact > 0 {
        eprintln!(
            "  [reloc] executor ticks: fw/act={qn_act} rot/goal={qn_goal} stop/noact={qn_noact}"
        );
    }
    r
}

fn main() -> ExitCode {
    let args = Args::parse();
    let modes: Vec<Mode> = {
        let mut v = Vec::new();
        for name in args.modes.split(',').map(str::trim).filter(|s| !s.is_empty()) {
            let Some(m) = Mode::from_name(name) else {
                eprintln!("error: unknown mode {name}");
                return ExitCode::from(2);
            };
            v.push(m);
        }
        v
    };
    let kidnaps: Vec<(String, Kidnap)> = {
        let mut v = Vec::new();
        for s in &args.kidnap {
            match parse_kidnap(s, args.tick_s) {
                Ok(k) => v.push(k),
                Err(e) => {
                    eprintln!("error: bad --kidnap {s}: {e}");
                    return ExitCode::from(2);
                }
            }
        }
        v
    };
    let map_path = args.map.clone().unwrap_or_else(default_map_path);
    eprintln!("loading map: {}", map_path.display());
    let map = match pgm::load(&map_path) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("error: failed to load map: {e}");
            return ExitCode::from(2);
        }
    };
    let full_res = map.meta.resolution;
    let res = full_res * args.scale as f64;
    let unknown_obst = args.unknown == UnknownMode::Obstacle;
    let (occ, ow, oh) = pgm::build_occupancy(&map, args.scale, unknown_obst);
    let (nocc, nw_, nh_) = pgm::build_occupancy(&map, 1, unknown_obst);
    eprintln!(
        "grid: {ow}x{oh}x{N_THETA} @ {res:.3} m/cell (native {nw_}x{nh_} @ {full_res:.3})"
    );

    let goal_x = args.goal_x.unwrap_or(map.meta.origin_x + map.width as f64 * full_res / 2.0);
    let goal_y = args.goal_y.unwrap_or(map.meta.origin_y + map.height as f64 * full_res / 2.0);
    let req_gx = (((goal_x - map.meta.origin_x) / res).floor() as i32).clamp(0, ow - 1);
    let req_gy = (((goal_y - map.meta.origin_y) / res).floor() as i32).clamp(0, oh - 1);
    let Some((gx, gy)) = snap_to_free(&occ, ow, oh, req_gx, req_gy, ow.max(oh)) else {
        eprintln!("error: no free cell near goal");
        return ExitCode::from(2);
    };
    let goal_wx = map.meta.origin_x + (gx as f64 + 0.5) * res;
    let goal_wy = map.meta.origin_y + (gy as f64 + 0.5) * res;
    let goal_radius = args.goal_radius_m.unwrap_or((2.0 * res).max(0.5));
    eprintln!("goal: ({goal_wx:.2}, {goal_wy:.2}) r={goal_radius:.2} m");

    let grid = OccupancyGrid {
        width: ow,
        height: oh,
        resolution: res,
        origin_x: map.meta.origin_x,
        origin_y: map.meta.origin_y,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: occ.clone(),
    };
    let native = OccupancyGrid {
        width: nw_,
        height: nh_,
        resolution: full_res,
        origin_x: map.meta.origin_x,
        origin_y: map.meta.origin_y,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: nocc,
    };

    let actions = scaled_actions(args.action_scale);
    let max_fw = actions.iter().map(|a| a.delta_fw).fold(0.0f64, f64::max);
    if max_fw < 2.0 * res - 1e-9 {
        eprintln!(
            "warning: max forward step {max_fw:.2} m < 2 cells ({:.2} m) — value propagation may \
             degenerate; consider --action-scale {:.1}",
            2.0 * res,
            2.0 * res / 0.3
        );
    }
    let mut vi = ValueIterator::new(actions, 1);
    vi.set_map_with_occupancy_grid(
        &grid,
        N_THETA,
        args.safety_radius_m,
        args.safety_penalty,
        goal_radius,
        args.goal_margin_theta_deg,
    );
    vi.set_goal(goal_wx, goal_wy, args.goal_theta_deg as i32);

    let Some(solver) = U64Solver::from_name(&args.solver) else {
        eprintln!("error: unknown solver {}", args.solver);
        return ExitCode::from(2);
    };
    eprintln!("solving with {} ...", args.solver);
    let t0 = Instant::now();
    let stats = solve(&mut vi, solver, args.max_iters);
    eprintln!(
        "solved: iters={} {:.1} s converged={}",
        stats.iters,
        t0.elapsed().as_secs_f64(),
        if stats.converged { "Y" } else { "N" }
    );

    let bc = BeliefConfig {
        half_m: args.belief_half_m,
        sensor_sigma_m: args.belief_sigma_m,
        beam_step: args.beam_step,
        max_range_m: args.scan_range,
        motion_sigma_xy_m: args.motion_sigma_xy,
        motion_sigma_theta_deg: args.motion_sigma_theta_deg,
        init_sigma_xy_m: args.init_sigma_xy,
        init_sigma_theta_deg: args.init_sigma_theta_deg,
        contract_ess: args.contract_ess,
        reloc_scale: args.reloc_scale,
        ..BeliefConfig::default()
    };
    // 全地図 belief 側の同じノブ (窓の半径だけが無い)。
    let wbc = WholeBeliefConfig {
        sensor_sigma_m: args.belief_sigma_m,
        beam_step: args.beam_step,
        max_range_m: args.scan_range,
        motion_sigma_xy_m: args.motion_sigma_xy,
        motion_sigma_theta_deg: args.motion_sigma_theta_deg,
        init_sigma_xy_m: args.init_sigma_xy,
        init_sigma_theta_deg: args.init_sigma_theta_deg,
        viterbi: args.viterbi,
        lost_ess: args.lost_ess,
        contract_ess: args.contract_ess,
        reloc_scale: args.reloc_scale,
        lost_update_min_d_m: args.lost_min_d,
        lost_update_min_a_deg: args.lost_min_a_deg,
        ..WholeBeliefConfig::default()
    };
    {
        let probe = GridLocalizer::new(&native, N_THETA, bc);
        let whole = Belief::new(&grid, N_THETA, &native, wbc);
        eprintln!(
            "belief: grid {:.1} MB (window) / whole-map {:.1} MB ({} free cells)",
            probe.belief_mb(),
            whole.belief_mb(),
            whole.free_cells()
        );
    }

    // 誘拐先の妥当性 (free であること) を先に検証する。
    for (label, k) in &kidnaps {
        let (ix, iy, it) = pose_to_cell(&vi, k.x, k.y, k.yaw_rad);
        if !in_field(&vi, ix, iy, it) || !CostView::free_at(&vi, ix, iy) {
            eprintln!("error: kidnap {label} destination ({}, {}) is not free", k.x, k.y);
            return ExitCode::from(2);
        }
    }

    // スタート乱択 (follow_ctrl_bench と同じ: free・障害物 2 セル以上・到達可能)。
    // --start-x/--start-y があれば固定スタートを --trials 回。
    let chamfer = chamfer_dist(&occ, ow, oh);
    let mut rng = Rng(args.seed.max(1));
    let mut starts: Vec<(f64, f64, f64)> = Vec::new();
    if let (Some(sx), Some(sy)) = (args.start_x, args.start_y) {
        let yaw = args.start_theta_deg.to_radians();
        let (ix, iy, it) = pose_to_cell(&vi, sx, sy, yaw);
        if !in_field(&vi, ix, iy, it) || vi.value_at(ix, iy, it) >= MAX_COST {
            eprintln!("error: fixed start ({sx}, {sy}) has no policy (unreachable or blocked)");
            return ExitCode::from(2);
        }
        starts = vec![(sx, sy, yaw); args.trials.max(1)];
    }
    let mut attempts = 0u64;
    while starts.len() < args.starts && args.start_x.is_none() && attempts < 2_000_000 {
        attempts += 1;
        let ix = rng.below(ow as u64) as i32;
        let iy = rng.below(oh as u64) as i32;
        if occ[(iy * ow + ix) as usize] != 0 || chamfer[(iy * ow + ix) as usize] < 6 {
            continue;
        }
        let x = map.meta.origin_x + (ix as f64 + 0.5) * res;
        let y = map.meta.origin_y + (iy as f64 + 0.5) * res;
        let dist = ((x - goal_wx).powi(2) + (y - goal_wy).powi(2)).sqrt();
        if dist < args.min_start_m || dist > args.max_start_m {
            continue;
        }
        let yaw_deg = rng.below(360) as f64;
        let it = ((yaw_deg / vi.t_resolution).floor() as i32).clamp(0, N_THETA - 1);
        if vi.value_at(ix, iy, it) >= MAX_COST {
            continue;
        }
        starts.push((x, y, yaw_deg.to_radians()));
    }
    if starts.is_empty() {
        eprintln!("error: no valid start found");
        return ExitCode::from(2);
    }

    // シナリオ: 誘拐なしの base + 各誘拐先。--kidnap なしなら従来どおり base のみ。
    let scenarios: Vec<(String, Option<Kidnap>)> = if kidnaps.is_empty() {
        vec![("base".to_string(), None)]
    } else {
        std::iter::once(("base".to_string(), None))
            .chain(kidnaps.iter().map(|(l, k)| (l.clone(), Some(*k))))
            .collect()
    };

    fn outcome_str(r: &RunResult) -> String {
        if r.reached {
            "reached".to_string()
        } else if r.believed_goal {
            format!("GOAL_BEL({:.2} m)", r.final_err_m)
        } else if r.collided {
            "COLLIDED".to_string()
        } else if r.out_of_map {
            "OUT_OF_MAP".to_string()
        } else if r.lost_gave_up {
            "LOST".to_string()
        } else if r.starved {
            "NO_ACTION".to_string()
        } else {
            "TIMEOUT".to_string()
        }
    }

    let mut results: Vec<(usize, String, RunResult)> = Vec::new();
    for (si, &start) in starts.iter().enumerate() {
        // シードのずらし (方向乱択、モード間で共通)。
        let phi = rng.unit() * 2.0 * std::f64::consts::PI;
        let seed_pose = PoseView {
            x: start.0 + args.seed_offset_m * phi.cos(),
            y: start.1 + args.seed_offset_m * phi.sin(),
            yaw_rad: start.2
                + args.seed_offset_deg.to_radians() * if rng.below(2) == 0 { 1.0 } else { -1.0 },
        };
        let noise_seed = rng.next();
        let dist = ((start.0 - goal_wx).powi(2) + (start.1 - goal_wy).powi(2)).sqrt();
        println!(
            "start {si} @ ({:.2}, {:.2}, {:.0} deg) dist={dist:.1} m",
            start.0,
            start.1,
            start.2.to_degrees()
        );
        for (label, kd) in &scenarios {
            for &mode in &modes {
                let t_run = Instant::now();
                let mut traj_buf = args
                    .traj_dir
                    .as_ref()
                    .map(|_| String::from("t,x,y,yaw,est_x,est_y,est_yaw,quality,ess\n"));
                let r = simulate(
                    &mut vi,
                    &grid,
                    &native,
                    &chamfer,
                    mode,
                    start,
                    (goal_wx, goal_wy),
                    seed_pose,
                    bc,
                    wbc,
                    noise_seed,
                    *kd,
                    solver,
                    goal_radius,
                    traj_buf.as_mut(),
                    &args,
                );
                if let (Some(dir), Some(buf)) = (&args.traj_dir, traj_buf) {
                    let _ = std::fs::create_dir_all(dir);
                    let p = dir.join(format!("traj_{label}_{}_{si}.csv", mode.name()));
                    if let Err(e) = std::fs::write(&p, buf) {
                        eprintln!("warning: failed to write traj: {e}");
                    }
                }
                let recovery = if r.kidnapped {
                    let reloc = if r.reloc_solves > 0 {
                        format!(
                            "  reloc solve={:.1}s x{} drive={:.1}m",
                            r.reloc_solve_s, r.reloc_solves, r.reloc_dist_m
                        )
                    } else {
                        String::new()
                    };
                    format!(
                        "  det={} lock={} lost_us={:.0}/{:.0}{}",
                        r.detect_tick
                            .map(|t| format!("{:.1}s", t as f64 * args.tick_s))
                            .unwrap_or_else(|| "-".into()),
                        r.relock_tick
                            .map(|t| format!("{:.1}s", t as f64 * args.tick_s))
                            .unwrap_or_else(|| "-".into()),
                        r.lost_us_sum / r.lost_us_ticks.max(1) as f64,
                        r.lost_us_max,
                        reloc,
                    )
                } else {
                    String::new()
                };
                println!(
                    "  {label:5}/{:8}: {:16} ticks={:5} ({:6.1} s)  len={:6.1} m  clr min={:4.2}/avg={:4.2} m  err rms={:5.3}/max={:5.3} m  yaw rms={:5.1} deg  qual={:4.2}  loc={:7.1}/{:.0} us{}  [wall {:.0} s]",
                    r.mode,
                    outcome_str(&r),
                    r.ticks,
                    r.ticks as f64 * args.tick_s,
                    r.path_len_m,
                    r.clear_min_m,
                    r.clear_mean_m(),
                    r.err_rms_m(),
                    r.err_max_m,
                    r.yaw_rms_deg(),
                    r.quality_mean(),
                    r.loc_us_sum / r.ticks.max(1) as f64,
                    r.loc_us_max,
                    recovery,
                    t_run.elapsed().as_secs_f64(),
                );
                results.push((si, label.clone(), r));
            }
        }
    }

    println!();
    println!("| spec | mode | reach | bel | bel_err_m | ticks | time_s | len_m | clr_min_m | clr_avg_m | err_rms_m | err_max_m | yaw_rms_deg | quality | loc_us | max_us |");
    println!("|------|------|-------|-----|-----------|-------|--------|-------|-----------|-----------|-----------|-----------|-------------|---------|--------|--------|");
    for (label, _) in &scenarios {
        for &mode in &modes {
            let all: Vec<&RunResult> = results
                .iter()
                .filter(|(_, l, r)| l == label && r.mode == mode.name())
                .map(|(_, _, r)| r)
                .collect();
            let ok: Vec<&&RunResult> = all.iter().filter(|r| r.reached).collect();
            // 到達扱い (truth final or 信じて停止) の走行。bel_err は後者の真値誤差平均。
            let bel: Vec<&&RunResult> =
                all.iter().filter(|r| r.believed_goal && !r.reached).collect();
            println!(
                "| {label} | {} | {}/{} | {} | {:.2} | {:.0} | {:.1} | {:.1} | {:.2} | {:.2} | {:.3} | {:.3} | {:.1} | {:.2} | {:.1} | {:.0} |",
                mode.name(),
                ok.len(),
                all.len(),
                bel.len(),
                mean(bel.iter().map(|r| r.final_err_m)),
                mean(ok.iter().map(|r| r.ticks as f64)),
                mean(ok.iter().map(|r| r.ticks as f64 * args.tick_s)),
                mean(ok.iter().map(|r| r.path_len_m)),
                all.iter().map(|r| r.clear_min_m).fold(f64::INFINITY, f64::min),
                mean(all.iter().map(|r| r.clear_mean_m())),
                mean(all.iter().map(|r| r.err_rms_m())),
                all.iter().map(|r| r.err_max_m).fold(0.0f64, f64::max),
                mean(all.iter().map(|r| r.yaw_rms_deg())),
                mean(all.iter().map(|r| r.quality_mean())),
                mean(all.iter().map(|r| r.loc_us_sum / r.ticks.max(1) as f64)),
                all.iter().map(|r| r.loc_us_max).fold(0.0f64, f64::max),
            );
        }
    }

    // 復帰集計 (誘拐シナリオがあるときだけ)。extra_* は同モードの base
    // (誘拐なし・到達走行) との差 — 誘拐が走行全体に上乗せしたコスト。
    if !kidnaps.is_empty() {
        println!();
        println!("| spec | mode | reach | detect | t_detect_s | relock | t_relock_s | lost_us_avg | lost_us_max | reloc_solve_s | reloc_drive_m | extra_time_s | extra_len_m |");
        println!("|------|------|-------|--------|------------|--------|------------|-------------|-------------|---------------|---------------|--------------|-------------|");
        for (label, _) in kidnaps.iter() {
            for &mode in &modes {
                let all: Vec<&RunResult> = results
                    .iter()
                    .filter(|(_, l, r)| l == label && r.mode == mode.name())
                    .map(|(_, _, r)| r)
                    .collect();
                let ok = all.iter().filter(|r| r.reached).count();
                let det: Vec<f64> = all
                    .iter()
                    .filter_map(|r| r.detect_tick)
                    .map(|t| t as f64 * args.tick_s)
                    .collect();
                let lock: Vec<f64> = all
                    .iter()
                    .filter_map(|r| r.relock_tick)
                    .map(|t| t as f64 * args.tick_s)
                    .collect();
                // 終端成功 = truth final か believed-goal 停止 (実ノードの成功宣言)。
                // ゴール半径ちょうど外の GOAL_BEL(0.5 m) を弾かないため後者も含める。
                let done = |r: &RunResult| r.reached || r.believed_goal;
                let base: Vec<&RunResult> = results
                    .iter()
                    .filter(|(_, l, r)| l == "base" && r.mode == mode.name() && (r.reached || r.believed_goal))
                    .map(|(_, _, r)| r)
                    .collect();
                let reached: Vec<&&RunResult> = all.iter().filter(|r| done(r)).collect();
                let (extra_t, extra_len) = if base.is_empty() || reached.is_empty() {
                    (f64::NAN, f64::NAN)
                } else {
                    (
                        mean(reached.iter().map(|r| r.ticks as f64 * args.tick_s))
                            - mean(base.iter().map(|r| r.ticks as f64 * args.tick_s)),
                        mean(reached.iter().map(|r| r.path_len_m))
                            - mean(base.iter().map(|r| r.path_len_m)),
                    )
                };
                println!(
                    "| {label} | {} | {}/{} | {}/{} | {:.1} | {}/{} | {:.1} | {:.0} | {:.0} | {:.1} | {:.1} | {:.1} | {:.1} |",
                    mode.name(),
                    ok,
                    all.len(),
                    det.len(),
                    all.len(),
                    mean(det.iter().copied()),
                    lock.len(),
                    all.len(),
                    mean(lock.iter().copied()),
                    mean(all.iter().map(|r| r.lost_us_sum / r.lost_us_ticks.max(1) as f64)),
                    all.iter().map(|r| r.lost_us_max).fold(0.0f64, f64::max),
                    mean(all.iter().map(|r| r.reloc_solve_s)),
                    mean(all.iter().map(|r| r.reloc_dist_m)),
                    extra_t,
                    extra_len,
                );
            }
        }
    }

    if let Some(path) = &args.out {
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                let _ = std::fs::create_dir_all(parent);
            }
        }
        let mut csv = String::from(
            "start,spec,mode,outcome,final_err_m,ticks,time_s,len_m,clr_min_m,clr_avg_m,err_rms_m,err_max_m,yaw_rms_deg,quality,loc_us_avg,loc_us_max,t_detect_s,t_relock_s,lost_us_avg,lost_us_max,reloc_solve_s,reloc_solves,reloc_dist_m\n",
        );
        for (si, label, r) in &results {
            let outcome = if r.reached {
                "reached"
            } else if r.believed_goal {
                "goal_bel"
            } else if r.collided {
                "collided"
            } else if r.out_of_map {
                "out_of_map"
            } else if r.lost_gave_up {
                "lost"
            } else if r.starved {
                "no_action"
            } else {
                "timeout"
            };
            let opt_s =
                |t: Option<usize>| t.map(|t| format!("{:.2}", t as f64 * args.tick_s)).unwrap_or_default();
            csv.push_str(&format!(
                "{si},{label},{},{outcome},{:.3},{},{:.1},{:.2},{:.3},{:.3},{:.4},{:.4},{:.2},{:.3},{:.2},{:.1},{},{},{:.1},{:.1},{:.2},{},{:.2}\n",
                r.mode,
                r.final_err_m,
                r.ticks,
                r.ticks as f64 * args.tick_s,
                r.path_len_m,
                r.clear_min_m,
                r.clear_mean_m(),
                r.err_rms_m(),
                r.err_max_m,
                r.yaw_rms_deg(),
                r.quality_mean(),
                r.loc_us_sum / r.ticks.max(1) as f64,
                r.loc_us_max,
                opt_s(r.detect_tick),
                opt_s(r.relock_tick),
                r.lost_us_sum / r.lost_us_ticks.max(1) as f64,
                r.lost_us_max,
                r.reloc_solve_s,
                r.reloc_solves,
                r.reloc_dist_m,
            ));
        }
        if let Err(e) = std::fs::write(path, csv) {
            eprintln!("warning: failed to write CSV: {e}");
        } else {
            eprintln!("csv written to {}", path.display());
        }
    }
    ExitCode::SUCCESS
}
