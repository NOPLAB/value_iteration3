//! follow 1 tick の判断器 ([`FollowController`]) — 離散 greedy (本家 `decision`
//! 準拠) と連続 DWA ([`vi_reference::ctrl`]) を trait で抽象化し、
//! [`PlanConfig::follow_controller`] で切り替える。
//!
//! どちらも読む場は同じ (`PlannerCore::local` が返す密な `ValueIterator` — 密経路は
//! 全域、compact 経路はハイドレート済みパッチ) で、ゴール判定 (`final_state`) と
//! [`Decision`] の意味論も共通。違うのは「場から速度指令を引く読み方」だけなので、
//! solve・ロールアウト・全域掃きには一切影響しない。
//!
//! - [`GreedyController`] — 現在セルの離散方策 (無ければ近傍借用)。本家
//!   `ViNode::decision` そのもの。既定。
//! - [`DwaController`] — V̂ 三線形補間 + (v, ω) 候補格子の軌道サンプリング
//!   (評価は衝突棄却 + 終端 V̂ のみ — 理由は `vi_reference::ctrl` の doc)。候補
//!   全滅 (現在セル非 free / 全候補衝突 / 終端評価不能) 時は greedy へフォール
//!   バックするので、失敗の形は greedy と同じに保たれる。実測
//!   (`follow_ctrl_bench`, 津田沼 scale 3): 同到達率 6/6 でコマンド変動 Σ|Δω|
//!   半減・到達 5% 短縮、decide は ~30 µs (40 ms 予算の 0.1%)。
//! - [`MppiController`] — MPPI 型 (名目制御列へのガウス摂動 + softmax 重み付き
//!   平均、warm start)。評価規約は DWA と同一。乱数は決定的で、tick 間状態
//!   (名目列) は `Mutex` の内部可変性で持つ (`decide` は共有ロック下で `&self`)。
//!   フォールバック時は名目列を捨てる — 場に合っていない列を次 tick に
//!   持ち越さないため。
//!
//! compact 経路との整合: DWA/MPPI のホライズン (既定 1.0 s × v_max 0.3 m/s =
//! 0.3 m) は ±1 m ウィンドウの内側に収まり、パッチ外へ出る候補は `value_at` が
//! `MAX_COST` を返して自然に棄却される (凍結境界の不変条件はそのまま)。

use std::sync::Mutex;

use vi_reference::bridge::PoseView;
use vi_reference::ctrl::{dwa_decide, mppi_decide, DwaConfig, MppiConfig, MppiState};
use vi_reference::planner::pose_to_cell;
use vi_reference::value_iterator::ValueIterator;
use vi_reference::Action;

use super::{action_at, is_final, Decision, PlanConfig};

/// [`PlanConfig::follow_controller`] の選択肢 (ROS パラメータ `follow_controller`)。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FollowKind {
    /// 本家 `ViNode::decision` 準拠の離散 greedy (既定)。
    Greedy,
    /// 連続行動 (V̂ 補間 + DWA 型軌道サンプリング、greedy フォールバック付き)。
    Dwa,
    /// 連続行動 (V̂ 補間 + MPPI 型サンプリング、greedy フォールバック付き)。
    Mppi,
}

impl FollowKind {
    pub fn from_name(s: &str) -> Option<Self> {
        Some(match s {
            "greedy" => FollowKind::Greedy,
            "dwa" => FollowKind::Dwa,
            "mppi" => FollowKind::Mppi,
            _ => return None,
        })
    }
}

/// follow 1 tick の判断器。`vi` は solve 済みの密な局所場。返す
/// [`Decision::Action`] の `(fw, rot_deg)` は速度指令 (`linear.x [m/s]` /
/// `angular.z [deg/s]`) としてそのまま配信される (本家 `decision` の読み方)。
///
/// 状態を持つ実装 (MPPI の warm start 等) は内部可変性で持つこと — `decide` は
/// 共有ロックの下で `&self` で呼ばれる。
pub trait FollowController: Send {
    fn name(&self) -> &'static str;
    fn decide(&self, vi: &ValueIterator, cfg: &PlanConfig, pose: PoseView) -> Decision;
}

/// 設定から判断器を組む ([`super::PlannerCore::new`] が呼ぶ)。
pub(super) fn make_controller(cfg: &PlanConfig, actions: &[Action]) -> Box<dyn FollowController> {
    match cfg.follow_controller {
        FollowKind::Greedy => Box::new(GreedyController),
        FollowKind::Dwa => Box::new(DwaController::new(cfg, actions)),
        FollowKind::Mppi => Box::new(MppiController::new(cfg, actions)),
    }
}

/// 本家 `ViNode::decision` の `posToAction` 相当。現在セルに方策が無ければ同一 θ の
/// 近傍 (チェビシェフ距離 `action_tolerance_cells` 以内) から最近傍の行動 /
/// ゴールセルを借りる。
pub struct GreedyController;

impl FollowController for GreedyController {
    fn name(&self) -> &'static str {
        "greedy"
    }

    fn decide(&self, vi: &ValueIterator, cfg: &PlanConfig, pose: PoseView) -> Decision {
        let (ix, iy, it) = pose_to_cell(vi, pose.x, pose.y, pose.yaw_rad);

        if is_final(vi, ix, iy, it) {
            return Decision::Goal;
        }
        if let Some(d) = action_at(vi, ix, iy, it) {
            return d;
        }

        let tol = cfg.action_tolerance_cells;
        let mut best: Option<(i64, Decision)> = None;
        for dy in -tol..=tol {
            for dx in -tol..=tol {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let (nx, ny) = (ix + dx, iy + dy);
                let cand = if is_final(vi, nx, ny, it) {
                    Some(Decision::Goal)
                } else {
                    action_at(vi, nx, ny, it)
                };
                let Some(cand) = cand else { continue };
                let d2 = (dx as i64) * (dx as i64) + (dy as i64) * (dy as i64);
                if best.as_ref().map(|(bd, _)| d2 < *bd).unwrap_or(true) {
                    best = Some((d2, cand));
                }
            }
        }
        best.map(|(_, d)| d).unwrap_or(Decision::NoAction)
    }
}

/// 連続行動: `vi_reference::ctrl::dwa_decide` の薄いラッパ。
pub struct DwaController {
    dwa: DwaConfig,
}

impl DwaController {
    pub fn new(cfg: &PlanConfig, actions: &[Action]) -> Self {
        // 速度範囲は行動集合から (v = delta_fw [m/s]、ω = delta_rot [deg/s] —
        // greedy が出す指令と同じ範囲なので、機体側の上限を新たに越えない)。
        let mut dwa = DwaConfig::from_actions(actions, cfg.dwa_tick_s.max(1e-3));
        dwa.horizon_s = cfg.dwa_horizon_s.max(cfg.dwa_tick_s);
        dwa.n_v = cfg.dwa_n_v.max(2);
        dwa.n_w = cfg.dwa_n_w.max(3);
        dwa.lethal_penalty =
            (cfg.dwa_lethal_penalty.max(0.0) * vi_reference::params::PROB_BASE as f64) as u64;
        Self { dwa }
    }
}

impl FollowController for DwaController {
    fn name(&self) -> &'static str {
        "dwa"
    }

    fn decide(&self, vi: &ValueIterator, cfg: &PlanConfig, pose: PoseView) -> Decision {
        let (ix, iy, it) = pose_to_cell(vi, pose.x, pose.y, pose.yaw_rad);
        if is_final(vi, ix, iy, it) {
            return Decision::Goal;
        }
        if let Some(c) = dwa_decide(vi, vi, &self.dwa, pose.x, pose.y, pose.yaw_rad) {
            return Decision::Action { id: None, fw: c.v, rot_deg: c.w_deg };
        }
        // 候補全滅: 従来の離散 greedy (近傍借用込み) で救済する。膨張域の縁に
        // 掛かった・パッチの縁で評価不能、のような場面でロボットを止めないため。
        GreedyController.decide(vi, cfg, pose)
    }
}

/// 連続行動 (MPPI): `vi_reference::ctrl::mppi_decide` のラッパ。tick 間状態
/// (warm start の名目制御列 + 乱数) は `Mutex` の内部可変性で持つ — `decide` は
/// 共有ロックの下で `&self` で呼ばれるため。この Mutex は follow ループの
/// 1 tick 内でしか取られないので競合しない。
pub struct MppiController {
    mppi: MppiConfig,
    state: Mutex<MppiState>,
}

impl MppiController {
    pub fn new(cfg: &PlanConfig, actions: &[Action]) -> Self {
        let mut mppi = MppiConfig::from_actions(actions, cfg.dwa_tick_s.max(1e-3));
        mppi.horizon_s = cfg.dwa_horizon_s.max(cfg.dwa_tick_s);
        mppi.n_samples = cfg.mppi_samples.max(2);
        mppi.lambda = cfg.mppi_lambda;
        if cfg.mppi_sigma_v > 0.0 {
            mppi.sigma_v = cfg.mppi_sigma_v;
        }
        if cfg.mppi_sigma_w_deg > 0.0 {
            mppi.sigma_w_deg = cfg.mppi_sigma_w_deg;
        }
        let state = Mutex::new(MppiState::new(mppi.seed));
        Self { mppi, state }
    }
}

impl FollowController for MppiController {
    fn name(&self) -> &'static str {
        "mppi"
    }

    fn decide(&self, vi: &ValueIterator, cfg: &PlanConfig, pose: PoseView) -> Decision {
        let (ix, iy, it) = pose_to_cell(vi, pose.x, pose.y, pose.yaw_rad);
        if is_final(vi, ix, iy, it) {
            return Decision::Goal;
        }
        let mut state = self.state.lock().unwrap();
        if let Some(c) = mppi_decide(vi, vi, &self.mppi, &mut state, pose.x, pose.y, pose.yaw_rad)
        {
            return Decision::Action { id: None, fw: c.v, rot_deg: c.w_deg };
        }
        // 候補全滅: 名目列は場に合っていないので捨てて (次の成功 tick で作り
        // 直される)、DWA と同じく離散 greedy で救済する。
        state.reset();
        drop(state);
        GreedyController.decide(vi, cfg, pose)
    }
}
