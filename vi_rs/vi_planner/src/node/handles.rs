//! ノードが持つ共有物 — 核・自己位置推定・スキャン待ち行列・パブリッシャ。
//!
//! ここにあるものは**ゴールが変わっても作り直さない**。ゴールごとに変わるもの
//! (cancel フラグ・進捗の宛先) はアクションサーバ側 ([`super::servers`]) が持つ。

use std::sync::Mutex;
use std::time::{Duration, Instant};

use vi_lib::bridge::PoseView;
use vi_lib::msg::LaserScan as ViLaserScan;

use vi_planner::core::{Belief, Localizer, PlannerCore};

use rclrs::*;

use super::follow_loop::FollowTuning;
use super::msg::ros_grid_from;

// ──────────────────────────────────────────────────────────────────────────────
// Value function visualization
// ──────────────────────────────────────────────────────────────────────────────

/// 可視化配信一式。`value_function` は両アクションの solve が共有する θ=0 全域
/// スライス (価値関数は 1 本しかないので、旧 vi_local_planner の
/// `local_value_function` に相当するトピックは無い)。
pub struct Viz {
    /// θ=0 全域スライス (solve の途中経過 + 完了時 + 追従中の伝播の進み具合)。
    /// 追従中に出しているのは掃きスレッドで、`global_sweep: false` だと
    /// solve した瞬間のまま固まる。
    pub vf_pub: Publisher<nav_msgs::msg::OccupancyGrid>,
    /// ローカルウィンドウの現在方位スライス (追従中、スキャン penalty 込み)。
    pub win_pub: Publisher<nav_msgs::msg::OccupancyGrid>,
    /// 自己位置 belief の θ 周辺分布 (`localizer` が内蔵推定器のときだけ中身が出る)。
    pub belief_pub: Publisher<nav_msgs::msg::OccupancyGrid>,
    /// belief 配信の間引き状態。価値関数と違って「収束」イベントが無く、
    /// シードとスキャン補正の 2 箇所から呼ばれるので Viz 側に持たせる。
    pub belief_last: Mutex<Option<Instant>>,
    pub clock: Clock,
    pub frame_id: String,
    /// カラースケールの上限 [ステップ数≒秒] (`value_function` と
    /// `local_window_value` で共通)。
    pub threshold_steps: u64,
    /// 配信間隔。0 で solve 完了時のみ。
    pub interval: Duration,
}

impl Viz {
    pub fn stamp(&self) -> (i32, u32) {
        self.clock.now().to_sec_nanosec().unwrap_or((0, 0))
    }

    /// 間引き判定。`last` を更新した場合のみ true。
    pub fn due(&self, last: &mut Option<Instant>) -> bool {
        if self.interval.is_zero() {
            return false;
        }
        if last.map_or(false, |t| t.elapsed() < self.interval) {
            return false;
        }
        *last = Some(Instant::now());
        true
    }
}

/// 表示専用の経路パブリッシャ (`plan`)。Nav2 構成ではこれを出すのは
/// planner_server (= `compute_path_to_pose` の側) で、RViz の Path 表示や
/// `daifuku_rqt` が見ているのはそのトピック。スタンドアロンでは
/// `navigate_to_pose` が誰も `compute_path_to_pose` を呼ばないので、
/// ここから出さないと**画面に経路が 1 本も出ない**。
///
/// あくまで表示専用で、ロールアウトが失敗しても走行には影響しない
/// (追従は経路ではなく方策を 1 手ずつ引く)。
pub struct PlanPub {
    pub path_pub: Publisher<nav_msgs::msg::Path>,
    pub clock: Clock,
    pub frame_id: String,
}

// ──────────────────────────────────────────────────────────────────────────────
// Localizer
// ──────────────────────────────────────────────────────────────────────────────

/// 自己位置推定。3 系統が併存する:
///   - `External` — pose トピックの素通し (既定)。
///   - `Windowed` — 窓つきヒストグラム MCL ([`vi_planner::core::GridLocalizer`] /
///     [`vi_planner::core::AdaptiveLocalizer`])。belief は機体まわりの窓だけに載り、
///     `AdaptiveLocalizer` は観測が合わなくなると粗い広域レベルへ広げて再定位する
///     (能動的再定位の判別点 `reloc_targets` を出せるのもこちら)。
///   - `Belief` — 全地図 belief ([`vi_planner::core::Belief`])。窓もレベル機構も無く、
///     プランナと同じ格子の上に belief を全域で持つ。
///
/// **`PlannerCore` の外に置くこと。** 30 秒級の solve は核の `Mutex` を握りっぱなし
/// にするが、belief はその間もスキャンコールバックから更新され続けなければならない。
/// ロック順序は常に localizer → core (入れ子は作らない)。
pub enum Loc {
    External(Option<PoseView>),
    Windowed(Box<dyn Localizer>),
    Belief(Box<Belief>),
}

impl Loc {
    /// 外部姿勢の取り込み。External = そのまま採用、他は手動シード。
    pub fn set_pose(&mut self, p: PoseView) {
        match self {
            Loc::External(o) => *o = Some(p),
            Loc::Windowed(l) => l.set_pose(p),
            Loc::Belief(b) => b.seed(p),
        }
    }
    pub fn predict(&mut self, v: f64, w_deg: f64, dt: f64) {
        match self {
            Loc::External(_) => {}
            Loc::Windowed(l) => l.predict(v, w_deg, dt),
            Loc::Belief(b) => b.predict(v, w_deg, dt),
        }
    }
    /// `map_clear` = `map_clear_from_scan`。VI 側と同じ反証を belief の free
    /// マスクにも当ててから補正する (順序が逆だと、開いた先へ質量を動かすのは
    /// 次の tick になる)。Windowed 側は未対応 — 必要になったら足す。
    pub fn observe(&mut self, scan: &ViLaserScan, map_clear: bool) {
        match self {
            Loc::External(_) => {}
            Loc::Windowed(l) => l.observe(scan),
            Loc::Belief(b) => {
                if map_clear {
                    b.clear_free_from_scan(scan);
                }
                b.observe(scan);
            }
        }
    }
    pub fn pose(&self) -> Option<PoseView> {
        match self {
            Loc::External(o) => *o,
            Loc::Windowed(l) => l.pose(),
            Loc::Belief(b) => b.pose(),
        }
    }
    /// 直近の補正の観測一致度 [0,1]。External は常に 1.0 (ゲートが実質無効)。
    pub fn quality(&self) -> f64 {
        match self {
            Loc::External(_) => 1.0,
            Loc::Windowed(l) => l.quality(),
            Loc::Belief(b) => b.quality(),
        }
    }
    /// QMDP 用の上位仮説。External は単一仮説なので空 = 呼び出し側は点推定へ退避。
    pub fn top_cells(&self, k: usize) -> Vec<(PoseView, f64)> {
        match self {
            Loc::External(_) => Vec::new(),
            Loc::Windowed(l) => l.top_cells(k),
            Loc::Belief(b) => b.top_cells(k),
        }
    }
    /// 能動的再定位 (`active_reloc`) の行き先候補。多峰 belief を持つ推定器
    /// (窓つき [`vi_planner::core::AdaptiveLocalizer`] と全地図
    /// [`vi_planner::core::Belief`]) が出す — External は空 = 提案なし。
    pub fn reloc_targets(&self) -> Vec<(f64, f64)> {
        match self {
            Loc::External(_) => Vec::new(),
            Loc::Windowed(l) => l.reloc_targets(),
            Loc::Belief(b) => b.reloc_targets(),
        }
    }
    /// 可視化用の belief グリッド。External は belief を持たないので None
    /// (窓つきは窓ぶんだけ、全地図は VI と同じ格子)。
    pub fn belief_grid(&self) -> Option<vi_lib::msg::OccupancyGrid> {
        match self {
            Loc::External(_) => None,
            Loc::Windowed(l) => l.belief_grid(),
            Loc::Belief(b) => b.grid(),
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Handles
// ──────────────────────────────────────────────────────────────────────────────

/// 4 つのアクションサーバ・購読コールバック・掃きスレッドが共有するハンドル束。
/// 各サーバはこれを 1 つ clone するだけでよい (以前は 6〜10 個の Arc を
/// closure 用と内側 async 用に二重 clone していた)。
pub struct Handles {
    pub core: Mutex<PlannerCore>,
    pub latest_pose: Mutex<Option<PoseView>>,
    /// 自己位置推定 ([`Loc`])。`latest_pose` はこれの出力キャッシュ — 読む側
    /// (follow ループ・plan サーバ) は従来どおり latest_pose だけを見る。
    /// **核の Mutex とは別**: solve 中も scan から更新され続ける必要がある。
    pub localizer: Mutex<Loc>,
    pub scan_queue: Mutex<Vec<ViLaserScan>>,
    pub cmd_pub: Publisher<geometry_msgs::msg::Twist>,
    pub viz: Option<Viz>,
    /// 表示専用の経路 (`plan`)。standalone のときだけ Some。
    pub plan_pub: Option<PlanPub>,
    /// 推定姿勢の表示用出力 (`viola_pose`)。シードとスキャン補正のたびに出す。
    pub est_pub: Publisher<geometry_msgs::msg::PoseStamped>,
    pub est_frame: String,
    pub est_clock: Clock,
    /// map→odom TF (`publish_tf`)。odom が届くたびに latest_pose と合成して出す
    /// (odom レートで出すので、スキャン間も TF が新鮮なまま)。
    pub tf_pub: Option<Publisher<tf2_msgs::msg::TFMessage>>,
    pub tf_tolerance: Duration,
}

impl Handles {
    /// 推定姿勢を `viola_pose` へ。
    pub fn publish_est(&self, p: PoseView) {
        let mut msg = geometry_msgs::msg::PoseStamped::default();
        msg.header.frame_id = self.est_frame.as_str().into();
        let (sec, nanosec) = self.est_clock.now().to_sec_nanosec().unwrap_or((0, 0));
        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nanosec;
        msg.pose.position.x = p.x;
        msg.pose.position.y = p.y;
        msg.pose.orientation.z = (p.yaw_rad / 2.0).sin();
        msg.pose.orientation.w = (p.yaw_rad / 2.0).cos();
        let _ = self.est_pub.publish(msg);
    }

    /// belief を `belief` トピックへ (シードとスキャン補正のたび、
    /// `value_publish_interval_ms` で間引き)。描画は数百セル〜格子 1 枚ぶんの
    /// 仕事なので、推定器のロックは取り直して短く握る。
    pub fn publish_belief(&self) {
        let Some(v) = self.viz.as_ref() else { return };
        if !v.due(&mut v.belief_last.lock().unwrap()) {
            return;
        }
        let Some(g) = self.localizer.lock().unwrap().belief_grid() else { return };
        let _ = v.belief_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
    }

    /// map→odom = T_map→base(推定) · T_odom→base⁻¹ を `/tf` へ (AMCL の契約)。
    /// スキャン時刻より新しい参照にも答えられるよう、スタンプは
    /// transform_tolerance だけ未来へ日付ける (AMCL と同じ手当て)。
    pub fn publish_tf(&self, est: PoseView, odo: PoseView, odom_frame: &str) {
        let Some(tf_pub) = self.tf_pub.as_ref() else { return };
        let th = est.yaw_rad - odo.yaw_rad;
        let (s, c) = th.sin_cos();
        let mut t = geometry_msgs::msg::TransformStamped::default();
        t.header.frame_id = self.est_frame.as_str().into();
        t.child_frame_id = odom_frame.into();
        let (sec, nanosec) = self.est_clock.now().to_sec_nanosec().unwrap_or((0, 0));
        let total =
            sec as i64 * 1_000_000_000 + nanosec as i64 + self.tf_tolerance.as_nanos() as i64;
        t.header.stamp.sec = (total / 1_000_000_000) as i32;
        t.header.stamp.nanosec = (total % 1_000_000_000) as u32;
        t.transform.translation.x = est.x - (c * odo.x - s * odo.y);
        t.transform.translation.y = est.y - (s * odo.x + c * odo.y);
        t.transform.rotation.z = (th / 2.0).sin();
        t.transform.rotation.w = (th / 2.0).cos();
        let mut msg = tf2_msgs::msg::TFMessage::default();
        msg.transforms = vec![t];
        let _ = tf_pub.publish(msg);
    }

    /// 追従ループ用の借用ビュー。`with_plan: false` は Nav2 構成の follow_path
    /// (表示用の `plan` は BT 側の compute_path_to_pose が出す)。
    pub fn follow_ctx(&self, tuning: FollowTuning, with_plan: bool) -> FollowCtx<'_> {
        FollowCtx {
            core: &self.core,
            latest_pose: &self.latest_pose,
            localizer: &self.localizer,
            scan_queue: &self.scan_queue,
            cmd_pub: &self.cmd_pub,
            plan_pub: if with_plan { self.plan_pub.as_ref() } else { None },
            viz: self.viz.as_ref(),
            tuning,
        }
    }
}

/// 追従ループが触る ROS 側の口。ゴールごとに変わらないものをまとめてある。
/// 全フィールドが参照か Copy なので、分配束縛のために Copy にしてある。
#[derive(Clone, Copy)]
pub struct FollowCtx<'a> {
    pub core: &'a Mutex<PlannerCore>,
    pub latest_pose: &'a Mutex<Option<PoseView>>,
    pub localizer: &'a Mutex<Loc>,
    pub scan_queue: &'a Mutex<Vec<ViLaserScan>>,
    pub cmd_pub: &'a Publisher<geometry_msgs::msg::Twist>,
    /// 表示専用の経路。None なら出さない (`follow_path` 構成では BT 側の
    /// `compute_path_to_pose` が出すので不要)。
    pub plan_pub: Option<&'a PlanPub>,
    pub viz: Option<&'a Viz>,
    pub tuning: FollowTuning,
}
