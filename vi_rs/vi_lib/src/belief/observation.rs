//! Beam likelihood scoring shared by sum-product and min-plus updates.
use super::{BeliefConfig, LikelihoodField, LOST_MIN_KNOWN, UNKNOWN_L};

pub(super) struct BeamPose {
    pub(super) position: (f64, f64),
    pub(super) heading: (f64, f64),
}

pub(super) struct ObservationModel {
    beams: Vec<(f64, f64, f64)>,
    z_min: f64,
    inverse_beams: f64,
    unknown: f64,
    global_match: bool,
    anneal_pow: i32,
}

impl ObservationModel {
    pub(super) fn new(cfg: &BeliefConfig, beams: &[(f64, f64)], anneal_pow: i32) -> Self {
        Self {
            beams: beams.iter().map(|&(a, r)| (a.cos(), a.sin(), r)).collect(),
            z_min: cfg.z_min,
            inverse_beams: 1.0 / beams.len() as f64,
            unknown: cfg.z_min + (1.0 - cfg.z_min) * UNKNOWN_L,
            global_match: cfg.global_match,
            anneal_pow,
        }
    }

    /// Return the quality and update weight, before either update rule applies it.
    pub(super) fn score(
        &self,
        field: &LikelihoodField,
        pose: BeamPose,
        lut: Option<&[[f32; 256]]>,
    ) -> (f64, f64) {
        let z_min = self.z_min;
        let m_inv = self.inverse_beams;
        let apow = self.anneal_pow;
        let unk_l = self.unknown;
        let bt = &self.beams;
        let (cx, cy) = pose.position;
        let (ct, st) = pose.heading;
        let (mut prod, mut known) = (1.0f64, 0u32);
        for (bi, &(cb, sb, r)) in bt.iter().enumerate() {
            let (ca, sa) = (ct * cb - st * sb, st * cb + ct * sb);
            let (px, py) = (cx + r * ca, cy + r * sa);
            match lut {
                // ロスト観測モデル: LUT は z_min ミキシング + (1/M)
                // テンパリング焼き込み済み。未知・地図外の端点は分母から
                // 除外 (E 正規化 — LOST_MIN_KNOWN の doc 参照)。
                Some(t) => {
                    if let Some(d) = field.known_dist(px, py) {
                        prod *= t[bi][d] as f64;
                        known += 1;
                    }
                }
                None => {
                    let mut lb = z_min + (1.0 - z_min) * field.at(px, py);
                    // 未知セルの端点は中立 (UNKNOWN_L) との max。
                    if lb < unk_l && field.unk_at(px, py) {
                        lb = unk_l;
                    }
                    prod *= lb;
                }
            }
        }
        // 観測一致度はビームの**幾何**平均 (= prod^(1/M))。算術平均だと
        // ミスマッチでも「たまたま障害物帯に乗った端点」の寄与で 0.3 台に
        // 浮き、ロスト検出のしきい値と分離できない。幾何平均は外れビームに
        // 引きずられて z_min 側へ落ちるので、整合 (~0.5+) と乖離する。
        let (gm, bw) = if lut.is_some() {
            let s = if known >= LOST_MIN_KNOWN {
                // (Π l^(1/M))^(M/E) = (Π l)^(1/E) — 既知ビームの幾何平均。
                prod.powf(bt.len() as f64 / known as f64)
            } else {
                unk_l
            };
            // global_match: 淘汰の**重み**は「未知ビーム = 中立」で数え
            // 直し、焼きなまし乗する: (Π_known l^(1/M) · unk_l^((M−E)/M))^apow。
            // E 正規化の gm は少数ビーム候補ほど高分散で、良く合う数ビーム
            // の縁セル (gm ~0.9) が全証拠の真値 (gm ~0.7) を恒常的に上回る
            // (津田沼 K4 の北東フリンジで実測 — probation q=0.63 の高品質
            // エイリアスの正体)。中立記数なら証拠が薄いほど主張も薄く、
            // ゼロ証拠セルのタダ乗り (乗数 1) も起きない。quality (検出・
            // probation のしきい値系) は E 正規化 gm のまま。
            let bwv = if self.global_match {
                (prod * unk_l.powf((bt.len() - known as usize) as f64 * m_inv))
                    .powf(apow as f64)
            } else {
                s
            };
            (s, bwv)
        } else {
            (prod.powf(m_inv), prod)
        };
        (gm, bw)
    }
}
