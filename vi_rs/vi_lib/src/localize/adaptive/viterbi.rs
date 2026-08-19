//! [`AdaptiveLocalizer`] の全域レベルを min-plus (tropical) 半環で回す Viterbi 定位。
//!
//! sum-product のベイズフィルタ (predict = 確率シフト + ぼかし、correct = 尤度乗算)
//! を log 空間の min-plus に置き換える: δ(s) = 「s で終わる最良軌跡の累積 -ln 尤度」。
//! 更新は
//!
//!   δ ← min-plus シフト (溜めた cmd_vel) → 緩和 (指数型運動ノイズ) → +観測コスト
//!
//! で、これは HMM の Viterbi 前進再帰そのもの。窓付きの履歴リプレイは要らない —
//! 全域レベルに**入った時点で δ を初期化**すれば、再帰が「ロスト以降の全スキャン +
//! 全移動」の MAP を実質的に持ち回る (誘拐の前後でオドメトリが不連続になる問題も、
//! 履歴がロスト時点から始まることで構造的に消える)。
//!
//! sum-product 比の利点:
//! - 粗セル + ぼかしで拡散するベイズと違い最良軌跡の鋭さが保たれる (似た場所が
//!   複数ある地図では運動整合性が偽仮説を削る)。
//! - log 空間なので f32 subnormal 系の事故 (0×∞ = NaN) が構造的に無い。
//! - ロスト中の predict は移動量の記録だけ O(1) — 全域 2D 面の毎 tick シフトが
//!   消える (シフトは次の observe がまとめて適用)。
//!
//! 既存機構との橋: observe のたび b = exp(δmin − δ) を実体化して正規化するので、
//! ess / top_modes / contract / quality のレベル遷移機構はそのまま動く (このモードの
//! b は「MAP までの近さ」の擬似 belief)。
//!
//! 設計メモ: この min-plus 掃引はステンシル reduce の形が VI の Bellman 掃引と
//! 同型 — per-cell 演算の差し替えだけでストリーミング HLS カーネルに載る形を
//! 保っている (1 つの遷移モデル、2 つの半環)。

use super::*;

/// 運動ノイズの min-plus 緩和コスト [nats/セル]: 経路が 1 セル逸れるごとに
/// e^{-λ} の尤度比を払う指数型ノイズ (sum-product 側の blur に対応)。
// ponytail: 定数 2 個。地図・センサごとの調整が要るなら BeliefConfig へ昇格。
const VIT_LAMBDA_XY: f32 = 4.0;
/// 同、θ 1 ビンあたり。
const VIT_LAMBDA_T: f32 = 4.0;

/// min-plus モードの全域レベル状態 ([`BeliefConfig::viterbi`] のときだけ確保)。
pub(super) struct VitState {
    /// 累積 -ln 尤度 (小さいほど良い)。非 free・枝刈りは +INF。
    pub(super) delta: Vec<f32>,
    /// 前回の observe から溜めた移動量 (predict はここに足すだけ)。
    pub(super) pend_f: f64,
    pub(super) pend_dt_deg: f64,
}

impl VitState {
    /// 最終 (全域) レベルの寸法で確保する。
    pub(super) fn new(levels: &[Level]) -> Self {
        let l = levels.last().unwrap();
        Self {
            delta: vec![0.0; (l.nx * l.ny * l.nt) as usize],
            pend_f: 0.0,
            pend_dt_deg: 0.0,
        }
    }
}

impl AdaptiveLocalizer {
    /// min-plus モードが今のレベルで駆動中か (= viterbi 設定 かつ 全域レベル滞在)。
    pub(super) fn vit_on(&self) -> bool {
        self.vit.is_some() && self.levels.len() > 1 && self.cur + 1 == self.levels.len()
    }

    /// 全域レベル進入時の δ 初期化: 今の b (射影 + 一様混合、正規化済み) を
    /// -ln で写す (min-plus は定数シフト不変なので正規化定数は気にしない)。
    /// b = 0 (非 free 等) は +INF。溜めた移動も捨てる — δ の履歴はここから始まる。
    pub(super) fn vit_enter(&mut self) {
        let n = self.n_active();
        let Some(s) = self.vit.as_mut() else { return };
        for i in 0..n {
            s.delta[i] = if self.b[i] > 0.0 {
                (-(self.b[i] as f64).ln()) as f32
            } else {
                f32::INFINITY
            };
        }
        s.pend_f = 0.0;
        s.pend_dt_deg = 0.0;
    }

    /// observe の min-plus 版 (全域レベル): 溜めた移動の適用 → 緩和 → 観測コスト
    /// 加算 → b = exp(δmin − δ) の実体化。戻り値は quality (従来と同じ
    /// 「前回 b 加重のビーム幾何平均尤度」— レベル遷移のしきい値系をそのまま使う)。
    /// 正規化・q_ewma・maybe_transition は呼び出し側 (observe) の共通処理。
    pub(super) fn vit_observe(&mut self, beams: &[(f64, f64)]) -> f64 {
        let (nx, ny, nt, res, t_res) = {
            let l = &self.levels[self.cur];
            (l.nx, l.ny, l.nt, l.res, l.t_res_deg)
        };
        let n = (nx * ny * nt) as usize;

        {
            let s = self.vit.as_mut().unwrap();
            let (pf, pt) = (s.pend_f, s.pend_dt_deg);
            s.pend_f = 0.0;
            s.pend_dt_deg = 0.0;
            // セル未満の移動は繰り越さず捨てる — 半セルの誤差は緩和が吸収する。
            if pf.abs() >= 0.5 * res || pt.abs() >= 0.5 * t_res {
                minplus_shift(&mut s.delta[..n], &mut self.tmp[..n], nx, ny, nt, res, t_res, pf, pt);
            }
            // 移動ゼロでも回す (sum-product 側が毎 predict で blur するのと同役)。
            minplus_relax(&mut s.delta[..n], nx, ny, nt, VIT_LAMBDA_XY, VIT_LAMBDA_T);
        }

        let z_min = self.cfg.z_min;
        // Bayes 側の weight_skip_ratio と同じ意味の枝刈り (δ は -ln 重み)。
        let thr_ln = (-(self.cfg.weight_skip_ratio.max(1e-30) as f64).ln()) as f32;
        let (ox, oy) = (self.field.ox, self.field.oy);
        let (wx0, wy0) = (self.wx0, self.wy0);
        let mut quality = 0.0f64;
        {
            let s = self.vit.as_mut().unwrap();
            let delta = &mut s.delta;
            let lvl = &self.levels[self.cur];
            let dmin0 = delta[..n].iter().cloned().fold(f32::INFINITY, f32::min);
            for it in 0..nt {
                let th = ((it as f64 + 0.5) * t_res).to_radians();
                for iy in 0..ny {
                    let cy = oy + (wy0 + iy) as f64 * res + res * 0.5;
                    for ix in 0..nx {
                        let i = bidx2(nx, ny, ix, iy, it);
                        let d = delta[i];
                        // NaN (全滅時の INF−INF) もこの否定形で落ちる。
                        if !(d - dmin0 <= thr_ln) {
                            delta[i] = f32::INFINITY;
                            continue;
                        }
                        if !level_free(&self.field, lvl, wx0, wy0, ix, iy) {
                            delta[i] = f32::INFINITY;
                            continue;
                        }
                        let cx = ox + (wx0 + ix) as f64 * res + res * 0.5;
                        let mut prod = 1.0f64;
                        for &(ba, r) in beams {
                            let a = th + ba;
                            let l = self.field.at(cx + r * a.cos(), cy + r * a.sin());
                            prod *= z_min + (1.0 - z_min) * l;
                        }
                        quality += self.b[i] as f64 * prod.powf(1.0 / beams.len() as f64);
                        delta[i] = d - prod.ln() as f32;
                    }
                }
            }
            // b = exp(δmin − δ) の実体化。
            let dmin = delta[..n].iter().cloned().fold(f32::INFINITY, f32::min);
            if dmin.is_finite() {
                for i in 0..n {
                    self.b[i] = if delta[i].is_finite() {
                        ((dmin - delta[i]) as f64).exp() as f32
                    } else {
                        0.0
                    };
                }
            } else {
                // 全滅 (シフトで地図外へ抜けた等) — free 一様へ自己修復。
                for iy in 0..ny {
                    for ix in 0..nx {
                        let f = level_free(&self.field, lvl, wx0, wy0, ix, iy);
                        for it in 0..nt {
                            let i = bidx2(nx, ny, ix, iy, it);
                            delta[i] = if f { 0.0 } else { f32::INFINITY };
                            self.b[i] = if f { 1.0 } else { 0.0 };
                        }
                    }
                }
            }
        }
        quality
    }
}

/// min-plus の決定的シフト: 各 θ 面をその方位の移動量ぶん整数シフトし、θ 面
/// 自体を回転ぶん円環シフトする (sum-product の predict の min-plus 版 — 補間は
/// しない。半セルの誤差は直後の緩和が吸収する)。範囲外からの取り込みは +INF。
fn minplus_shift(
    delta: &mut [f32],
    tmp: &mut [f32],
    nx: i32,
    ny: i32,
    nt: i32,
    res: f64,
    t_res: f64,
    pf: f64,
    pt_deg: f64,
) {
    for it in 0..nt {
        let th = ((it as f64 + 0.5) * t_res).to_radians();
        let rx = (pf * th.cos() / res).round() as i32;
        let ry = (pf * th.sin() / res).round() as i32;
        for iy in 0..ny {
            for ix in 0..nx {
                let (sx, sy) = (ix - rx, iy - ry);
                tmp[bidx2(nx, ny, ix, iy, it)] = if sx >= 0 && sx < nx && sy >= 0 && sy < ny {
                    delta[bidx2(nx, ny, sx, sy, it)]
                } else {
                    f32::INFINITY
                };
            }
        }
    }
    let rt = ((pt_deg / t_res).round() as i32).rem_euclid(nt);
    for it in 0..nt {
        let st = (it - rt).rem_euclid(nt);
        for iy in 0..ny {
            for ix in 0..nx {
                delta[bidx2(nx, ny, ix, iy, it)] = tmp[bidx2(nx, ny, ix, iy, st)];
            }
        }
    }
}

/// 軸分離の min-plus 緩和 (soft erosion): δ(s) ← min_k δ(s ± k·e) + λ·k。
/// 前進 + 後退の 2 掃引で軸ごとの距離変換になる (θ は円環なので 2 周する)。
fn minplus_relax(delta: &mut [f32], nx: i32, ny: i32, nt: i32, l_xy: f32, l_t: f32) {
    for it in 0..nt {
        for iy in 0..ny {
            for ix in 1..nx {
                let p = delta[bidx2(nx, ny, ix - 1, iy, it)] + l_xy;
                let i = bidx2(nx, ny, ix, iy, it);
                if p < delta[i] {
                    delta[i] = p;
                }
            }
            for ix in (0..nx - 1).rev() {
                let p = delta[bidx2(nx, ny, ix + 1, iy, it)] + l_xy;
                let i = bidx2(nx, ny, ix, iy, it);
                if p < delta[i] {
                    delta[i] = p;
                }
            }
        }
        for ix in 0..nx {
            for iy in 1..ny {
                let p = delta[bidx2(nx, ny, ix, iy - 1, it)] + l_xy;
                let i = bidx2(nx, ny, ix, iy, it);
                if p < delta[i] {
                    delta[i] = p;
                }
            }
            for iy in (0..ny - 1).rev() {
                let p = delta[bidx2(nx, ny, ix, iy + 1, it)] + l_xy;
                let i = bidx2(nx, ny, ix, iy, it);
                if p < delta[i] {
                    delta[i] = p;
                }
            }
        }
    }
    if nt > 1 {
        for iy in 0..ny {
            for ix in 0..nx {
                for k in 1..(2 * nt) {
                    let p = delta[bidx2(nx, ny, ix, iy, (k - 1).rem_euclid(nt))] + l_t;
                    let i = bidx2(nx, ny, ix, iy, k % nt);
                    if p < delta[i] {
                        delta[i] = p;
                    }
                }
                for k in (0..(2 * nt - 1)).rev() {
                    let p = delta[bidx2(nx, ny, ix, iy, (k + 1) % nt)] + l_t;
                    let i = bidx2(nx, ny, ix, iy, k % nt);
                    if p < delta[i] {
                        delta[i] = p;
                    }
                }
            }
        }
    }
}
