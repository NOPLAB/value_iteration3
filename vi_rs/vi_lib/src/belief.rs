//! 全地図 belief 推定器 — VIOLA の「belief を VI の状態へ」の推定側。
//!
//! 旧 範囲 belief (localize の GridLocalizer の窓 / AdaptiveLocalizer の
//! 多重解像度レベル) の後継。belief をプランナの [`crate::ValueIterator`] と
//! **同一の格子** (map_scale 適用後の nx×ny×θ) に全域で持つ — 窓の再センタ
//! リングもレベル遷移も無く、推定の状態空間と計画の状態空間が 1:1 に対応する
//! (QMDP・多目標 VI がセル添字のまま繋がる)。尤度場だけは生解像度地図
//! (native) から起こす — スキャン端点の評価は格子を粗くすると壊れるため。
//!
//! 旧実装からの置き換え:
//! - 窓/レベル + expand/contract カスケード → 全域 1 枚 + EMCL 風一様混合
//!   リセット ([`BeliefConfig::reset_quality`])。
//! - ロスト検出 (粗レベル滞在 = pose None) → ESS しきい値
//!   ([`BeliefConfig::lost_ess`]): belief の広がりの実測がしきい値を超えたら
//!   [`Belief::pose`] は `None`。
//! - 毎 tick の全面シフト predict → O(1) の移動量累積 + observe 冒頭の一括
//!   flush (アクティブセルだけの scatter)。全域を毎 tick 舐めない。
//! - viterbi (min-plus / MAP) はロスト時の特殊モードではなく全期間の更新則
//!   (旧 localize/adaptive/viterbi.rs の doc にある VI Bellman 掃引との同型性は
//!   そのまま)。
//!
//! アクティブ集合: `b[i] > 0` のセル添字を `active` に持ち、predict の flush・
//! 尤度・平均・ESS はすべて active だけを舐める。`weight_skip_ratio` の枝刈りが
//! 集合を belief の広がりに比例した大きさに保つ。

use crate::bridge::PoseView;
use crate::msg::{LaserScan, OccupancyGrid, Quaternion};

/// θ で周辺化済みの質量 `m` (w×h) → 可視化用 OccupancyGrid。最大質量を 98 と
/// する相対スケールで、価値関数の `value_grid_on` と同じく「そのまま Map 表示に
/// 流せる」形にする。
///
/// 質量ゼロは **0** (-1 ではない): RViz の costmap カラースキームで 0 だけが
/// 完全透過で、-1 は不透明の灰緑 — belief はほぼ全セルが質量ゼロなので、-1 に
/// すると地図の上に灰色の膜を張ってしまう。平方根を噛ませてあるのは、収束後の
/// belief が 1 セルに集中して副次モードが潰れる (= 多峰かどうかが画面で見えない)
/// のを避けるため。99/100 は同スキームの特別色なので使わない。
pub fn mass_to_grid(
    m: &[f32],
    w: i32,
    h: i32,
    res: f64,
    ox: f64,
    oy: f64,
    oq: Quaternion,
) -> OccupancyGrid {
    let max = m.iter().copied().fold(0.0f32, f32::max);
    OccupancyGrid {
        width: w,
        height: h,
        resolution: res,
        origin_x: ox,
        origin_y: oy,
        origin_quat: oq,
        data: m
            .iter()
            .map(|&p| if p > 0.0 { 1 + (97.0 * (p / max).sqrt()) as i8 } else { 0 })
            .collect(),
    }
}

/// [`Belief`] のチューニング。実機はスキャナも床も理想モデルからずれるので、
/// ノイズ幅は必ずパラメータで残す。
#[derive(Clone, Copy, Debug)]
pub struct BeliefConfig {
    /// 尤度場のガウス幅 [m] (スキャン端点と最近障害物の距離に対する σ)。
    pub sensor_sigma_m: f64,
    /// 補正に使うビームの間引き (1 = 全ビーム)。
    pub beam_step: usize,
    /// これより遠いレンジは補正に使わない [m] (無効レンジの差し替え値も弾く)。
    pub max_range_m: f64,
    /// predict 1 tick あたりの位置ノイズ σ [m]。
    pub motion_sigma_xy_m: f64,
    /// predict 1 tick あたりの方位ノイズ σ [deg]。
    pub motion_sigma_theta_deg: f64,
    /// [`Belief::seed`] (手動シード) で置く初期 belief の σ [m] / [deg]。
    pub init_sigma_xy_m: f64,
    pub init_sigma_theta_deg: f64,
    /// ビームごとの尤度の床 (完全ミスマッチでも重みを 0 にしない)。本家
    /// likelihood field モデルの z_rand/z_max 混合に相当する 1 定数。
    pub z_min: f64,
    /// 重みの相対しきい値 (max との比)。observe 後の枝刈り = 疎なアクティブ
    /// 集合のしきい値を兼ね、補正コストを belief の広がりに比例させる。
    pub weight_skip_ratio: f64,
    /// 観測一致度 EWMA がこれ未満なら observe 末尾で free 一様を
    /// [`MIX_UNIFORM`] だけ混合する (EMCL 風リセット — 旧 expand/contract
    /// カスケードの後継、レベルなし)。
    pub reset_quality: f64,
    /// ESS ([`Belief::ess`] = 1/Σb²) がこれ超で [`Belief::pose`] = None
    /// (= ロスト)。
    pub lost_ess: f64,
    /// ロスト解除 (再集中) とみなす ESS — これ未満でラッチが降りる。`lost_ess`
    /// と同じく**絶対セル数**なので、広い地図では両方を地図スケールに合わせて
    /// 上げる (進入/解除の 10 倍ヒステリシスを保つのが目安)。TB3 級の既定
    /// (500/50) はキャンパス級 (津田沼 0.15 m 格子) では健全な追跡状態の ESS
    /// (廊下方向の広がりで数百セル) が進入しきい値を超え、ラッチが永久に
    /// 降りなくなる。
    pub contract_ess: f64,
    /// 判別変位探索 ([`reloc_targets`]) の幾何スケール。基準 (1.0) は屋内向け
    /// (変位 1.5/3.0 m、署名リング 1.0 m)。クリアランス数 m 級の屋外地図では
    /// 尤度場の台 (~σ) にリングが届かず全候補スコア 0 になるので上げる
    /// (津田沼 0.15 m 格子で 4.0 が目安)。
    pub reloc_scale: f64,
    /// ロスト中の観測積分ゲート (AMCL の update_min_d / update_min_a 相当):
    /// 前回積分からの並進 [m] と回頭 [deg] が**両方**しきい値未満のスキャンは
    /// 読み捨てる。静止・微動のまま毎スキャン積分すると、強く相関した証拠を
    /// 独立扱いで指数的に過大計上し、判別機動が判別視点へ着く前に belief が
    /// 誤エイリアスへ単峰収束してしまう (津田沼で実測: 回頭 ~15 s で誤単峰)。
    /// 0 (既定) = ゲートなし = 従来挙動 — 静止反復観測での受動復帰 (一意な
    /// 幾何の小地図では有効) を保つ。使うときは両方をセットで。
    pub lost_update_min_d_m: f64,
    pub lost_update_min_a_deg: f64,
    /// 解除の probation (保護観察): 0 より大なら、ロスト解除を仮とし、
    /// `lost_update_min_d_m` の並進を伴う観測 probation_obs 回すべてで瞬時
    /// quality ≥ probation_min_q を確認してから確定する。1 回でも割れたら
    /// [`Belief::fail_probation`] (全域一様への再拡散) で lost へ戻す。
    /// 静止観測では誤エイリアスと真位置は判別できない (だから誤解除する —
    /// 津田沼で実測 err 170〜238 m、quality 0.5〜0.8) が、誤姿勢のまま走ると
    /// 予測が数 m で破綻する — その走行を検証に使う。0 (既定) = 従来 (即確定)。
    pub probation_obs: u32,
    pub probation_min_q: f64,
    /// ロスト中のみ、ビーム距離 r に比例して σ を膨らませる: σ_eff = σ0 +
    /// lost_sigma_per_m · r。仮説はセル中心 (±res/2) と θ ビン中心 (±3° @60
    /// ビン) でしか評価できず、その角度誤差は端点を接線方向に r·Δθ だけ振る —
    /// 60 m ビームで ±3 m。固定 σ0 = 0.2 m はこれを尤度 0 に落とすため、真位置
    /// のセルが長ビームで不当に罰され、たまたま整列した遠方エイリアスが崩壊に
    /// 勝つ (津田沼で実測: 誤エイリアスの瞬時 quality 0.88〜1.00)。非ロストは
    /// 加重平均でサブセル追跡できるので σ0 のまま — 検出感度も変えない。
    /// θ ビン半幅 3° = 0.052 rad が自然な値 (~0.05)。0 (既定) = 従来。
    ///
    /// 有効時はもう 1 つ、**スキャン内テンパリング**も同じ LUT に焼き込まれる:
    /// ビーム尤度の積を (1/M) 乗の積 = 幾何平均に置き換え、1 観測のダイナミック
    /// レンジを [z_min, 1] に圧縮する。生の積はビームあたり 25% 劣るだけの
    /// 仮説を 2 観測で相対枝刈りの下へ落とし、リセット直後の真位置仮説が
    /// 偶然の最良適合セルに 0.4 m で永久に殺される (winner's curse — 津田沼
    /// K4 で実測)。observe 内コメント参照。
    pub lost_sigma_per_m: f64,
    /// ロスト中の再シードを flatten (free 一様) でなく**全域相関スキャンマッチ**
    /// ([`Belief::match_reseed`] — Olson の correlative matching を 2 段に潰した
    /// もの) にする: ブロック σ 繰り込みの粗採点で粗ブロック × 全 θ を刈り、
    /// 生き残った枝ごとに真の σ の勝者セルへガウス塊を張る。flatten の「全
    /// free × θ がアクティブ」(キャンパス級で ~3 千万仮説、observe 数十秒)
    /// が「枝勝者の塊」(~10 万仮説、observe 数十 ms + マッチ 1 回 ~0.5 s) に
    /// なり、belief は追跡と検証 (probation) に専念する分業。候補に真値が
    /// 入らなくても quality 破綻 → 別視点のスキャンで再マッチするリトライ
    /// ループに落ちる。false (既定) = 従来の flatten。
    pub global_match: bool,
    /// min-plus (MAP / Viterbi) 更新則で回す。全期間 min-plus (レベル切替なし)。
    pub viterbi: bool,
}

impl Default for BeliefConfig {
    fn default() -> Self {
        Self {
            sensor_sigma_m: 0.2,
            beam_step: 10,
            max_range_m: 25.0,
            motion_sigma_xy_m: 0.03,
            motion_sigma_theta_deg: 2.0,
            init_sigma_xy_m: 0.3,
            init_sigma_theta_deg: 10.0,
            z_min: 0.05,
            weight_skip_ratio: 1e-4,
            reset_quality: 0.25,
            lost_ess: 500.0,
            contract_ess: 50.0,
            reloc_scale: 1.0,
            lost_update_min_d_m: 0.0,
            lost_update_min_a_deg: 0.0,
            probation_obs: 0,
            probation_min_q: 0.4,
            lost_sigma_per_m: 0.0,
            global_match: false,
            viterbi: false,
        }
    }
}

/// 観測一致度 EWMA の平滑係数。
const EWMA_BETA: f64 = 0.3;
/// リセットで free 一様分布と混ぜる質量比 (EMCL の resetting 相当)。
const MIX_UNIFORM: f32 = 0.5;
/// 未知セルに落ちた端点の中立尤度。1.0 (旧 = unknown を障害物扱い) は未知に
/// 囲まれた free の島が全ビーム満点になるブラックホール、0 (= 床) は世界側の
/// レイキャストが unknown 境界で終端するぶん**真値**が出血して base 追跡まで
/// 壊す (津田沼で両方とも実測)。追跡経路では実障害物の裾との max として、
/// ロスト観測モデルでは証拠不足仮説の頭打ち定数として使う。ponytail: 定数、
/// 地図ごとの調整が要るなら BeliefConfig へ昇格。
const UNKNOWN_L: f64 = 0.5;
/// ロスト観測モデルの最少証拠ビーム数 (E 正規化): 未知・地図外に落ちた端点は
/// 幾何平均の**分母から除外**し、既知端点ビームだけで評価する — 定数で数える
/// (中立クランプ) と、境界終端ビームを持つ真値が地図内エイリアスに恒常的に
/// 不利 (×0.8/観測、津田沼 K4 で実測: ゴーストに負け続けて枝刈り死)。既知
/// ビームがこれ未満の仮説は幾何平均を立てず UNKNOWN_L で頭打ち — 全ビームが
/// 未知に落ちる深部フリンジの偽者が「無罰 = 満点」へ戻るのを防ぐ。
const LOST_MIN_KNOWN: u32 = 4;
/// [`Belief::b_hat`] の下端アンカー: 「十分集中」とみなす ESS。
// ponytail: 定数、必要なら BeliefConfig へ昇格。
const TIGHT_ESS: f64 = 30.0;
/// 運動ノイズの min-plus 緩和コスト [nats/セル]: 経路が 1 セル逸れるごとに
/// e^{-λ} の尤度比を払う指数型ノイズ (sum-product 側の拡散に対応)。
// ponytail: 定数 2 個。地図・センサごとの調整が要るなら BeliefConfig へ昇格。
const VIT_LAMBDA_XY: f32 = 4.0;
/// 同、θ 1 ビンあたり。
const VIT_LAMBDA_T: f32 = 4.0;

/// belief の添字。θ 面優先 (`(it*ny + iy)*nx + ix`) — 移動シフトが θ 面ごとの
/// 連続 2D 面で回るように (旧 bidx2 レイアウト維持)。
#[inline]
fn bidx2(nx: i32, ny: i32, ix: i32, iy: i32, it: i32) -> usize {
    ((it * ny + iy) * nx + ix) as usize
}

/// scatter の書き込み + アクティブ候補の記録。scratch は使用間で全ゼロ不変
/// なので、初回タッチ (== 0.0) が重複なしの候補収集を兼ねる。
#[inline]
fn deposit(scratch: &mut [f32], cand: &mut Vec<u32>, j: usize, m: f32) {
    if m > 0.0 {
        if scratch[j] == 0.0 {
            cand.push(j as u32);
        }
        scratch[j] += m;
    }
}

/// [`Belief::reloc_targets`] の峰の取り方: 最大重みに対するしきい値、峰どうしの
/// 最小間隔 [m]、判別に使う峰の数 (窓つき側の `RELOC_MODES` と同値)、ソートを
/// 諦める候補数の上限。
const MODE_THRESHOLD: f32 = 0.05;
const MODE_MIN_SEP_M: f64 = 1.0;
const RELOC_MODES: usize = 4;
const MODE_CANDIDATE_CAP: usize = 4096;
/// ロスト解除の単峰判定に見る上位セル数 (QMDP に渡すのと同じ規模)。
const UNIMODAL_TOP_K: usize = 64;

/// 全域マッチャ ([`BeliefConfig::global_match`]) の定数: 粗探索ブロックの辺
/// [belief セル] / 粗枝 (ブロック × θ) の生存数。再シードは**枝ごとの勝者
/// 1 セル**を全部張る — 「全体 top-K セル」は高得点プラトー 1 領域が席を
/// 独占して真値が 1 席も取れない winner's curse の再来 (津田沼で実測:
/// 全 4 ケースで真値の重み 0 のまま) なので、空間多様性は枝の粒度で保証する。
// ponytail: 定数。地図規模で調整が要るなら BeliefConfig へ昇格。
const MATCH_STRIDE_CELLS: i32 = 8;
/// 粗枝の生存数。淘汰の頑健性 (真値が枝刈り線・候補落ちで死なない確率) は
/// 候補数に伸びる — 16384 枝 ≈ active 40 万セル ≈ flatten の 1.3% で、
/// observe は依然 ~0.2 s (4096 では真値の取りこぼしが残った、津田沼で実測)。
const MATCH_BRANCHES: usize = 16384;
/// global_match のロスト淘汰の証拠強度の上限: 観測の重み乗数を gm^pow に
/// する (quality は gm のまま)。pow は固定でなく**焼きなまし** — 再マッチ
/// からの走行距離 [`MATCH_ANNEAL_M`] ごとに 1 → この上限へ上がる。固定だと
/// 両立しない (津田沼で実測): 1 (テンパリングのみ) は勝者がノイズでフリップ
/// して解除のピーク保持が満ちず真値で凍結、8 固定は注入直後の真値が離散化
/// ノイズの増幅で即殺される (実効 σ が 1/√pow に狭まる — σ を √pow 広げると
/// ガウス部分が打ち消し合って中立化するだけ、これも実測)。「軟らかく開始
/// (注入直後を保護) → 走るほど硬化 (決着)」で、3 m のレート制限と噛み合う。
/// 上限 8 は gm=0.05 でも 4e-11 で f32 に収まる中庸。
const LOST_EVIDENCE_POW: i32 = 8;
/// 焼きなましの距離刻み [m]: pow = 1 + (前回マッチからの走行 / この値)。
const MATCH_ANNEAL_M: f64 = 0.5;
/// global_match の解除に要求するピーク保持: argmax ピークが 1 m 以内に
/// 留まったまま (ゲートを通った) 観測がこれだけ連続するまで解除しない。
/// 0.2 m ゲートなら ~2 m の creep に相当。誤ピークの勝者は再マッチ・淘汰の
/// たびにテレポートするのに対し真値ピークは何百 tick も動かない (津田沼で
/// 実測) — 一瞬の誤単峰に解除すると probation の 20〜60 m 誤走行が走行予算を
/// 吸収する。
const RELEASE_HOLD_OBS: u32 = 10;
/// 再マッチのレート制限 [m]: 前回マッチからこれだけ並進するまで、quality
/// 不一致でも belief を触らない (観測の淘汰に任せる)。毎不一致で再マッチすると
/// 反証済みエイリアスが毎回復活して勝者がテレポートを繰り返し、真値の蓄積
/// 優位が育たない (津田沼で実測 — flatten が機能したのは 3 千万セルの淘汰中
/// ESS ゲートが再リセットを長時間抑止していたからで、疎な候補集合は数観測で
/// ESS を割ってしまう)。creep 0.3 m/s + 0.2 m ゲートなら ~15 観測の淘汰窓。
const MATCH_RETRY_M: f64 = 3.0;
/// ローカル再マッチの探索半径 [m]: 追跡からの最初の不一致は「推定の近傍に
/// いるが滑った」一時破綻の可能性がまずある — その近傍のブロックだけで
/// 引き直す。検出遅れ (q_ewma) 中のドリフトを覆う値 (津田沼 K4 ルート対照
/// run の err max ~2.2 m)。
const LOCAL_REMATCH_R_M: f64 = 5.0;
/// ローカル再マッチの採用比: 近傍最良が全域最良のこの比以上ならローカル
/// 注入を選ぶ。**絶対しきい値にしないこと** — 破綻が起きる低特徴区間では
/// どのスコアも低い上、絶対線を一度でも超える誤アンカーは局所再注入で
/// 全域補正から永久に保護されてしまう (津田沼 K4 ルートで実測: 絶対線 0.5
/// で err max 130 m へ悪化)。比較なら、本物の誘拐では旧位置近傍が全域に
/// 大きく見劣りして自然に全域再シードへ落ちる。比は**寛容に**取る: 追跡中と
/// いう事前分布は強く、一時破綻の現場では近傍が真値スケール (~0.7) を出して
/// いても、地図のどこかに単発スキャン 0.98 の偶然エイリアスがほぼ常に存在
/// する (津田沼 K4 ルートで実測 — 0.8 比はそれに負けて全域注入 = churn に
/// 戻った)。誤ってローカルを取った場合の損失は 1 レート制限周期 (3 m) の
/// 遅延だけ (lost 分岐の再リセットは常に全域) で、誤って全域を取った場合の
/// 遠方エイリアス churn より桁違いに安い。
const LOCAL_REMATCH_ACCEPT: f32 = 0.5;

/// 3 点カーネル [a, 1-2a, a] の a。1 tick σ [セル] のランダムウォーク分散
/// (2a セル²) を合わせる (旧 GridLocalizer::blur_a)。累積 tick 分の a は
/// 呼び出し側が pass 分割で安定域 (中心重み非負) に収める — 旧 0.25
/// クランプの後継。
#[inline]
fn blur_a(sigma_cells: f64) -> f64 {
    sigma_cells * sigma_cells / 2.0
}

/// 静的地図から起こす 2D 尤度場: セルごとに exp(-d²/2σ²) (d = 最近障害物までの
/// 距離) を u8 量子化して持つ。チャンファー 2 パスなので構築は O(セル数)。
struct LikelihoodField {
    w: i32,
    h: i32,
    res: f64,
    ox: f64,
    oy: f64,
    lf: Vec<u8>,
    /// 最近障害物までのチャンファー距離 [セル]、255 で飽和 (native 0.05 m なら
    /// 12.75 m)。ロスト中の距離比例 σ ([`BeliefConfig::lost_sigma_per_m`]) は
    /// lf の u8 量子化 (d ≳ 3.5σ で 0 に潰れ復元不能) では評価できないため、
    /// 距離そのものを並置してビーム別 LUT で引く。
    dist: Vec<u8>,
    /// unknown (data < 0) セルの bitset。端点が落ちたら尤度を
    /// [`UNKNOWN_L`] でクランプ (実障害物の裾との max)。
    unk: Vec<u64>,
    /// free (data == 0) セルの bitset。belief の物理拘束用 — 壁・未知の中の
    /// 姿勢仮説を許さない (尤度場はビームの当たり先しか見ないので、これが
    /// 無いと「壁の中に居る」仮説が観測で一切罰されない)。
    free: Vec<u64>,
}

impl LikelihoodField {
    fn from_grid(g: &OccupancyGrid, sigma_m: f64) -> Self {
        let (w, h) = (g.width, g.height);
        let n = (w as usize) * (h as usize);
        // ROS 規約の三値: data > 0 = 障害物、0 = free、< 0 = unknown。
        // unknown は障害物にも free にも数えない — 障害物に数える (旧実装の
        // 「非 0 は障害物」) と、地図端の未知領域に囲まれた free の島がどんな
        // スキャンにも全ビーム尤度 1.0 で一致する「尤度場のブラックホール」に
        // なり、誘拐復帰の崩壊が毎回そこへ吸われる (津田沼で実測: 全誤解除の
        // 勝者が未知 49% の北東フリンジ、真位置は毎観測 ×0.5 で敗死)。
        // unknown に落ちた端点は最寄りの**実**障害物までの距離で評価され、
        // 深部なら床 z_min = 「情報なし」に落ちる (AMCL の likelihood_field が
        // 地図外端点を z_rand のみで評価するのと同じ側)。二値 {0,100} の
        // 呼び出し元には挙動不変。
        let mut d = vec![f32::INFINITY; n];
        let mut free = vec![0u64; n.div_ceil(64)];
        let mut unk = vec![0u64; n.div_ceil(64)];
        for i in 0..n {
            if g.data[i] > 0 {
                d[i] = 0.0;
            } else if g.data[i] == 0 {
                free[i >> 6] |= 1u64 << (i & 63);
            } else {
                unk[i >> 6] |= 1u64 << (i & 63);
            }
        }
        let idx = |x: i32, y: i32| (y * w + x) as usize;
        const DIAG: f32 = std::f32::consts::SQRT_2;
        // 前進パス (左上 → 右下)。
        for y in 0..h {
            for x in 0..w {
                let mut v = d[idx(x, y)];
                if x > 0 {
                    v = v.min(d[idx(x - 1, y)] + 1.0);
                }
                if y > 0 {
                    v = v.min(d[idx(x, y - 1)] + 1.0);
                    if x > 0 {
                        v = v.min(d[idx(x - 1, y - 1)] + DIAG);
                    }
                    if x < w - 1 {
                        v = v.min(d[idx(x + 1, y - 1)] + DIAG);
                    }
                }
                d[idx(x, y)] = v;
            }
        }
        // 後退パス (右下 → 左上)。
        for y in (0..h).rev() {
            for x in (0..w).rev() {
                let mut v = d[idx(x, y)];
                if x < w - 1 {
                    v = v.min(d[idx(x + 1, y)] + 1.0);
                }
                if y < h - 1 {
                    v = v.min(d[idx(x, y + 1)] + 1.0);
                    if x < w - 1 {
                        v = v.min(d[idx(x + 1, y + 1)] + DIAG);
                    }
                    if x > 0 {
                        v = v.min(d[idx(x - 1, y + 1)] + DIAG);
                    }
                }
                d[idx(x, y)] = v;
            }
        }
        let inv_2s2 = 1.0 / (2.0 * sigma_m * sigma_m);
        let dist = d.iter().map(|&dc| dc.min(255.0) as u8).collect();
        let lf = d
            .into_iter()
            .map(|dc| {
                let dm = dc as f64 * g.resolution;
                (255.0 * (-dm * dm * inv_2s2).exp()).round() as u8
            })
            .collect();
        Self { w, h, res: g.resolution, ox: g.origin_x, oy: g.origin_y, lf, dist, unk, free }
    }

    /// 世界座標が unknown セルか (追跡経路の [`UNKNOWN_L`] クランプ用)。地図外
    /// は false — 追跡では地図外端点を従来どおり床に落とす (base 追跡はそれで
    /// 成立しており、クランプを広げる理由がない)。
    #[inline]
    fn unk_at(&self, wx: f64, wy: f64) -> bool {
        let x = ((wx - self.ox) / self.res).floor() as i32;
        let y = ((wy - self.oy) / self.res).floor() as i32;
        if x < 0 || y < 0 || x >= self.w || y >= self.h {
            return false;
        }
        let i = (y * self.w + x) as usize;
        self.unk[i >> 6] & (1u64 << (i & 63)) != 0
    }

    /// 既知 (free または実障害物) セルに落ちた端点の距離インデックス。
    /// unknown と地図外は None = 「情報なし」— ロスト観測モデル
    /// ([`LOST_MIN_KNOWN`]) は幾何平均の分母から除外する。
    #[inline]
    fn known_dist(&self, wx: f64, wy: f64) -> Option<usize> {
        let x = ((wx - self.ox) / self.res).floor() as i32;
        let y = ((wy - self.oy) / self.res).floor() as i32;
        if x < 0 || y < 0 || x >= self.w || y >= self.h {
            return None;
        }
        let i = (y * self.w + x) as usize;
        if self.unk[i >> 6] & (1u64 << (i & 63)) != 0 {
            return None;
        }
        Some(self.dist[i] as usize)
    }

    /// セルが free か。地図外は false。
    #[inline]
    fn free_cell(&self, x: i32, y: i32) -> bool {
        if x < 0 || y < 0 || x >= self.w || y >= self.h {
            return false;
        }
        let i = (y * self.w + x) as usize;
        self.free[i >> 6] & (1u64 << (i & 63)) != 0
    }

    /// 世界座標のセルを free にする (scan 貫通による地図の反証)。地図外は無視。
    /// 尤度場 `lf` は触らない — 「壁があるはずの所を通った」は依然として尤度で
    /// 罰される。開くのは質量を置ける場所だけ。
    fn mark_free(&mut self, wx: f64, wy: f64) {
        let x = ((wx - self.ox) / self.res).floor() as i32;
        let y = ((wy - self.oy) / self.res).floor() as i32;
        if x < 0 || y < 0 || x >= self.w || y >= self.h {
            return;
        }
        let i = (y * self.w + x) as usize;
        self.free[i >> 6] |= 1u64 << (i & 63);
    }

    /// 世界座標が free セルに乗っているか。
    #[inline]
    fn free_at(&self, wx: f64, wy: f64) -> bool {
        self.free_cell(
            ((wx - self.ox) / self.res).floor() as i32,
            ((wy - self.oy) / self.res).floor() as i32,
        )
    }

    /// 世界座標の尤度 [0,1]。地図外は 0。
    fn at(&self, wx: f64, wy: f64) -> f64 {
        let x = ((wx - self.ox) / self.res).floor() as i32;
        let y = ((wy - self.oy) / self.res).floor() as i32;
        if x < 0 || y < 0 || x >= self.w || y >= self.h {
            return 0.0;
        }
        self.lf[(y * self.w + x) as usize] as f64 / 255.0
    }
}

/// 全域マッチャの粗探索プール (一度だけ構築)。
///
/// 当初は距離場のブロック min-pool (スコアの真の上界) を持っていたが、3.6 m
/// 窓の min は密集域で全ビーム距離 ~0 → 上界 ~1.0 の同点プラトーになり、
/// 同点タイブレークの枝選抜から真値が落ちる (津田沼で実測)。粗段は
/// ブロック中心の**実距離**をブロック半径ぶん膨らませた σ で採点する方式
/// (Olson の correlative matching の粗レベル標準) に変え、pool は free
/// ブロック一覧だけ残った。
struct MatchPool {
    /// free セルを 1 つ以上含む belief ブロックの原点 (belief セル座標)。
    /// ponytail: clear_free_from_scan が後から開けたセルは反映されない —
    /// 幽霊壁の開放で新たに free になるブロックはマッチ候補から漏れる。
    free_blocks: Vec<(i32, i32)>,
}

/// 全地図 belief ヒストグラム — 「belief を VI の状態へ」の推定側。
///
/// belief は VI と同一の格子 (vi_grid = map_scale 適用後の占有格子 × θ ビン)
/// に載る。旧 範囲 belief (GridLocalizer の 5m 窓 / AdaptiveLocalizer の
/// 多重解像度レベル) の後継で、窓オフセット (wx0/wy0) と Level 間接参照は
/// 存在しない — セル (ix, iy, it) は常に地図全域の絶対座標。
///
/// - predict は O(1) の移動量累積のみ。flush (シフト + 拡散) は observe 冒頭で
///   アクティブセルだけの scatter として一括適用する。flush 条件は
///   |pend_f| ≥ 2·res または |pend_rot_deg| ≥ t_res_deg 相当だが、実際の
///   適用点は observe 冒頭に統一 — [`Belief::pose`] は残 pend を平均へ解析的に
///   足すだけで済む (観測間の累積は上のしきい値オーダーに留まる)。
/// - ロスト = ESS > `lost_ess` **または** q_ewma < `reset_quality` (レベル
///   なし)。復帰は同じ q_ewma しきい値での free 一様混合 (EMCL 風リセット)。
/// - `viterbi: true` は同じ遷移モデルを min-plus 半環で回し、observe ごとに
///   b = exp(δmin − δ) を実体化するので ess/top_cells/quality は共通に動く。
pub struct Belief {
    cfg: BeliefConfig,
    /// native 解像度地図の 2D 尤度場 (θ なし・全域・構築 1 回)。
    field: LikelihoodField,
    /// belief 格子 = vi_grid の寸法・幾何。
    nx: i32,
    ny: i32,
    nt: i32,
    res: f64,
    t_res_deg: f64,
    ox: f64,
    oy: f64,
    /// 地図原点の回転 ([`Belief::grid`] が echo するだけ)。
    oq: Quaternion,
    /// vi_grid の free (data == 0) マスク (2D)。belief の物理拘束。
    free: Vec<bool>,
    n_free: usize,
    /// belief 本体 (レイアウトは [`bidx2`])。b[i] > 0 ⟺ i ∈ active。
    b: Vec<f32>,
    /// scatter の書き込み先。sum-product 経路では使用間で全ゼロ不変。
    scratch: Vec<f32>,
    /// アクティブセル添字 (疎な作業集合)。
    active: Vec<u32>,
    /// min-plus の累積 -ln 尤度 (viterbi 時のみ確保)。非 free・枝刈りは +INF。
    delta: Vec<f32>,
    /// 前回 flush から溜めた移動量 (predict はここに足すだけ)。
    pend_f: f64,
    pend_rot_deg: f64,
    pend_ticks: u32,
    initialized: bool,
    /// 直近 observe の belief 加重ビーム幾何平均尤度。
    quality: f64,
    q_ewma: f64,
    /// ESS = 1/Σb² のキャッシュ (b が変わる場所の末尾で更新 — pose() が
    /// 10 Hz で読むので毎回 active を舐めない)。
    ess_c: f64,
    /// ロスト状態のラッチ (旧 AdaptiveLocalizer の「レベル > 0」に相当)。
    ///
    /// 瞬時値の述語では表せない: 誘拐直後の belief は「間違った場所で自信
    /// 満々」なので ESS は小さいまま (ESS だけでは取りこぼす)、かつ観測不一致
    /// で発火する一様混合リセットが同じ observe 内で q_ewma を書き戻すので、
    /// observe から戻った時点では「合っていなかった」証拠も消えている。
    /// 立つ: 観測不一致 (q_ewma < reset_quality) か belief 拡大 (ess > lost_ess)。
    /// 降りる: 再集中 (ess < [`BeliefConfig::contract_ess`]) かつ観測が合っている
    /// (旧 AdaptiveLocalizer の contract 条件と同じ)、**かつ単峰**
    /// ([`mode_count`] ≤ 1 — 離れたエイリアスに分かれたままの「集中」で
    /// 降ろすと誤姿勢を返す。津田沼 K4 で実測)。
    lost: bool,
    /// 解除の probation の残り確認回数 (0 = probation 中でない) と、前回の
    /// 確認からの並進の累積 [m] ([`BeliefConfig::probation_obs`])。
    probation: u32,
    prob_move: f64,
    /// 全域マッチャの粗探索プール (global_match 時のみ、初回マッチで遅延構築)。
    match_pool: Option<MatchPool>,
    /// 前回の match_reseed からの並進 [m] ([`MATCH_RETRY_M`] のレート制限用)。
    move_since_match: f64,
    /// 解除のピーク保持カウンタとその anchor ([`RELEASE_HOLD_OBS`])。
    rel_hold: u32,
    rel_anchor: Option<(f64, f64)>,
}

impl Belief {
    /// `vi_grid` = プランナと同じ (map_scale 適用後の) 格子 — 幾何
    /// (nx/ny/res/origin) と free マスクの出どころ。`native` = 尤度場用の
    /// 生解像度地図。`theta_bins` は VI と同じ `theta_cell_num`。
    pub fn new(
        vi_grid: &OccupancyGrid,
        theta_bins: i32,
        native: &OccupancyGrid,
        cfg: BeliefConfig,
    ) -> Self {
        let field = LikelihoodField::from_grid(native, cfg.sensor_sigma_m);
        let (nx, ny) = (vi_grid.width, vi_grid.height);
        let nt = theta_bins.max(1);
        let n = (nx as usize) * (ny as usize) * (nt as usize);
        let free: Vec<bool> = vi_grid.data.iter().map(|&d| d == 0).collect();
        let n_free = free.iter().filter(|&&f| f).count();
        Self {
            cfg,
            field,
            nx,
            ny,
            nt,
            res: vi_grid.resolution,
            t_res_deg: 360.0 / nt as f64,
            ox: vi_grid.origin_x,
            oy: vi_grid.origin_y,
            oq: vi_grid.origin_quat.clone(),
            free,
            n_free,
            b: vec![0.0; n],
            scratch: vec![0.0; n],
            active: Vec::new(),
            delta: if cfg.viterbi { vec![f32::INFINITY; n] } else { Vec::new() },
            pend_f: 0.0,
            pend_rot_deg: 0.0,
            pend_ticks: 0,
            initialized: false,
            quality: 0.0,
            q_ewma: 0.0,
            ess_c: 0.0,
            lost: false,
            probation: 0,
            prob_move: 0.0,
            match_pool: None,
            move_since_match: 0.0,
            rel_hold: 0,
            rel_anchor: None,
        }
    }

    /// belief バッファのメモリ量 [MB] (起動ログ用)。
    pub fn belief_mb(&self) -> f64 {
        ((self.b.len() + self.scratch.len() + self.delta.len()) * 4) as f64 / 1e6
    }

    /// vi 格子の free セル数 (起動ログ / 一様混合の分母)。
    pub fn free_cells(&self) -> usize {
        self.n_free
    }

    /// 直近の補正の観測一致度 [0,1] (belief 加重のビーム幾何平均尤度)。
    pub fn quality(&self) -> f64 {
        self.quality
    }

    /// 有効セル数 ESS = 1/Σb² (正規化済み前提)。集中度の指標 (キャッシュ)。
    pub fn ess(&self) -> f64 {
        self.ess_c
    }

    /// ESS の対数ビニング: [`TIGHT_ESS`]〜`lost_ess` を log スケールで
    /// 0..nb-1 へ。ロスト ⇔ nb-1 と一貫させる (下の [`Belief::is_lost`] 参照)。
    pub fn b_hat(&self, nb: i32) -> i32 {
        if self.is_lost() {
            return (nb - 1).max(0);
        }
        let r = (self.ess_c / TIGHT_ESS).max(1e-300).ln() / (self.cfg.lost_ess / TIGHT_ESS).ln();
        ((nb as f64 * r).floor() as i32).clamp(0, nb - 1)
    }

    /// ロスト中か ([`Belief::lost`] ラッチの読み出し)。
    fn is_lost(&self) -> bool {
        self.lost
    }

    /// min-plus のリセット床の深さ λ [nats]: free 全セルへ δ ≤ δmin + λ を張る
    /// ときの λ。「一様成分の総質量 : ピーク = α : 1−α」(α = [`MIX_UNIFORM`],
    /// sum-product 側の混合比) から N·e^{−λ}/1 = α/(1−α) ⇒ λ = ln(N·α/(1−α))。
    ///
    /// 定数 λ にしてはいけない — 旧実装の 8.0 は粗レベル (~5k セル) 用の値で、
    /// 全地図 (~10⁶ セル) では一様側がピークの数百倍になり belief が永久に
    /// 平坦化する。枝刈りしきい値もこの λ を内側に含める必要がある
    /// ([`Belief::vit_observe`] 参照)。
    fn reset_floor_ln(&self) -> f64 {
        let n = (self.n_free * self.nt as usize) as f64;
        (n * MIX_UNIFORM as f64 / (1.0 - MIX_UNIFORM as f64)).max(1.0).ln()
    }

    /// θ ビン中心の方位 [rad] (ビン k は [k·res, (k+1)·res) 度 — 切り捨て規約に
    /// 合わせ、中心はその +0.5 ビン)。
    #[inline]
    fn theta_center(&self, it: i32) -> f64 {
        ((it as f64 + 0.5) * self.t_res_deg).to_radians()
    }

    /// セル中心の世界座標。
    #[inline]
    fn cell_center(&self, ix: i32, iy: i32) -> (f64, f64) {
        (
            self.ox + (ix as f64 + 0.5) * self.res,
            self.oy + (iy as f64 + 0.5) * self.res,
        )
    }

    /// 添字 → (ix, iy, it) ([`bidx2`] の逆)。
    #[inline]
    fn decode(&self, i: u32) -> (i32, i32, i32) {
        let plane = (self.nx as usize) * (self.ny as usize);
        let i = i as usize;
        let (it, r) = (i / plane, i % plane);
        ((r % self.nx as usize) as i32, (r / self.nx as usize) as i32, it as i32)
    }

    /// 手動シード (旧 set_pose 相当): belief を pose 中心のガウスで張り直す。
    /// 全地図なので窓の clamp・再センタリングは無い。
    pub fn seed(&mut self, pose: PoseView) {
        for &i in &self.active {
            self.b[i as usize] = 0.0;
        }
        self.active.clear();
        let s_xy = self.cfg.init_sigma_xy_m.max(self.res);
        let s_t = self.cfg.init_sigma_theta_deg.max(self.t_res_deg);
        // 分離ガウスの 1 次元重みを軸ごとに前計算し、無視できる行・列・ビンは
        // 飛ばす (全地図 nx·ny·nt ループを σ 窓に落とす。打ち切った成分は
        // 正規化後どのみち weight_skip_ratio の枝刈り域)。
        const CUT: f64 = 1e-9;
        let gx: Vec<(i32, f64)> = (0..self.nx)
            .filter_map(|ix| {
                let c = self.ox + (ix as f64 + 0.5) * self.res;
                let w = (-(c - pose.x).powi(2) / (2.0 * s_xy * s_xy)).exp();
                (w > CUT).then_some((ix, w))
            })
            .collect();
        let gy: Vec<(i32, f64)> = (0..self.ny)
            .filter_map(|iy| {
                let c = self.oy + (iy as f64 + 0.5) * self.res;
                let w = (-(c - pose.y).powi(2) / (2.0 * s_xy * s_xy)).exp();
                (w > CUT).then_some((iy, w))
            })
            .collect();
        let gt: Vec<(i32, f64)> = (0..self.nt)
            .filter_map(|it| {
                // 円環上の角度差。
                let d = (((it as f64 + 0.5) * self.t_res_deg) - pose.yaw_rad.to_degrees())
                    .rem_euclid(360.0);
                let dd = d.min(360.0 - d);
                let w = (-dd * dd / (2.0 * s_t * s_t)).exp();
                (w > CUT).then_some((it, w))
            })
            .collect();
        for &(it, wt) in &gt {
            for &(iy, wy) in &gy {
                let row = (iy * self.nx) as usize;
                for &(ix, wx) in &gx {
                    // 物理拘束: 非 free (壁・未知) に張らない (旧 apply_free_mask 相当)。
                    if !self.free[row + ix as usize] {
                        continue;
                    }
                    let w = (wt * wy * wx) as f32;
                    if w > 0.0 {
                        let i = bidx2(self.nx, self.ny, ix, iy, it);
                        self.b[i] = w;
                        self.active.push(i as u32);
                    }
                }
            }
        }
        self.normalize_active();
        if self.cfg.viterbi {
            self.vit_enter();
        }
        self.pend_f = 0.0;
        self.pend_rot_deg = 0.0;
        self.pend_ticks = 0;
        self.initialized = true;
        self.q_ewma = 1.0;
        // 手動シードは「ここに居る」という外部の主張 — ラッチを降ろす
        // (probation も外部の主張が上書きする)。
        self.lost = false;
        self.probation = 0;
        self.recompute_ess();
    }

    /// 大域初期化 / 全滅復旧: belief を free 一様で張る (旧 enter_uniform の
    /// 全域レベル版)。
    pub fn enter_uniform_free(&mut self) {
        for &i in &self.active {
            self.b[i as usize] = 0.0;
        }
        self.active.clear();
        for it in 0..self.nt {
            for iy in 0..self.ny {
                let row = (iy * self.nx) as usize;
                for ix in 0..self.nx {
                    if !self.free[row + ix as usize] {
                        continue;
                    }
                    let i = bidx2(self.nx, self.ny, ix, iy, it);
                    self.b[i] = 1.0;
                    self.active.push(i as u32);
                }
            }
        }
        self.normalize_active();
        if self.cfg.viterbi {
            self.vit_enter();
        }
        self.pend_f = 0.0;
        self.pend_rot_deg = 0.0;
        self.pend_ticks = 0;
        self.initialized = true;
        // リセット直後に再リセットしない (旧 enter_uniform の q_ewma 復帰と同じ)。
        self.q_ewma = self.cfg.reset_quality;
        self.lost = true;
        self.probation = 0;
        self.recompute_ess();
    }

    /// belief を**空**のロスト状態にする (global_match の再シード待ち)。次の
    /// observe が全滅復旧の経路でそのスキャンから [`Belief::match_reseed`] を
    /// 回す。flatten (enter_uniform_free) と違い、全 free × θ の flush/observe
    /// を一度も実体化しない — スキャンを持たない呼び出し元
    /// ([`Belief::fail_probation`]) の flatten 代替。
    fn enter_lost_empty(&mut self) {
        for &i in &self.active {
            self.b[i as usize] = 0.0;
        }
        self.active.clear();
        if self.cfg.viterbi {
            self.delta.fill(f32::INFINITY);
        }
        self.pend_f = 0.0;
        self.pend_rot_deg = 0.0;
        self.pend_ticks = 0;
        self.initialized = true;
        self.q_ewma = self.cfg.reset_quality;
        self.lost = true;
        self.probation = 0;
        self.ess_c = 0.0;
    }

    /// 予測 (実行した速度指令 1 tick ぶん): O(1) の累積のみ。実際のシフト +
    /// 拡散は observe 冒頭の flush が一括適用する。
    pub fn predict(&mut self, v: f64, w_deg: f64, dt: f64) {
        if !self.initialized {
            return;
        }
        self.pend_f += v * dt;
        self.pend_rot_deg += w_deg * dt;
        self.pend_ticks += 1;
    }

    /// 補正: flush(移動シフト + 拡散) → 尤度 → 枝刈り → 正規化 →
    /// q_ewma < reset_quality なら free 一様を混合。未シードなら大域初期化。
    pub fn observe(&mut self, scan: &LaserScan) {
        if !self.initialized {
            // 未シード: 最初のスキャンから大域一様で立ち上がる (大域初期化)。
            // global_match は一様を張らず、下の全滅復旧経路でスキャンマッチ
            // から立ち上がる。
            if self.cfg.global_match {
                self.enter_lost_empty();
            } else {
                self.enter_uniform_free();
            }
        }
        let lost = self.is_lost();
        // ロスト中の相関観測ゲート (config doc 参照): 前回積分から動いていない
        // スキャンは読み捨てる — flush もせず運動を貯め続ける。
        if lost
            && (self.cfg.lost_update_min_d_m > 0.0 || self.cfg.lost_update_min_a_deg > 0.0)
            && self.pend_f.abs() < self.cfg.lost_update_min_d_m
            && self.pend_rot_deg.abs() < self.cfg.lost_update_min_a_deg
        {
            return;
        }
        // probation の並進カウント用: この flush で消費する変位 (符号なし)。
        let moved_m = self.pend_f.abs();
        self.flush();
        self.move_since_match += moved_m;
        if self.active.is_empty() && !self.cfg.global_match {
            // flush で全質量が壁・地図外へ抜けた — 全滅復旧。global_match は
            // ビーム収集後にスキャンマッチで復旧する (下)。
            self.enter_uniform_free();
        }
        // ビーム収集 (`set_local_cost` と同じ世界角規約: ビーム角 = yaw +
        // angle_min + i·inc)。ロスト中は 4 倍間引き。相関観測ゲート有効時に
        // 「まれに全ビームで」も試したが、証拠を濃くすると誤エイリアスへの
        // 収束も速くなるだけだった (津田沼 K4 で実測 — 弁別できない曖昧性は
        // レートやビーム数では割れない) 上に observe が最大 23 s に膨らむ。
        // ponytail: ロスト中の observe は全域走査 — TB3 級で数十 ms/scan、
        // キャンパス級は秒単位。上限を上げるなら θ 間引き → coarse-to-fine
        // ゲートの順。global_match は active が疎な候補集合 (~10 万セル) なので
        // 間引きのコスト根拠が消える — 全ビームで弁別を取る (テンパリングは
        // 幾何平均なのでビーム数を変えてもレンジ圧縮は不変)。
        let step = self.cfg.beam_step.max(1)
            * if lost && !self.cfg.global_match { 4 } else { 1 };
        let max_r = self.cfg.max_range_m;
        let beams: Vec<(f64, f64)> = scan
            .ranges
            .iter()
            .enumerate()
            .step_by(step)
            .filter_map(|(i, &r)| {
                (r.is_finite() && r > 0.0 && r <= max_r)
                    .then(|| (scan.angle_min + scan.angle_increment * i as f64, r))
            })
            .collect();
        if beams.is_empty() {
            return;
        }
        if self.active.is_empty() {
            // global_match の全滅復旧 (fail_probation の enter_lost_empty も
            // ここに来る): flatten でなくスキャンマッチで再シードし、この
            // observe はそこで終える — マッチ自体がこのスキャンの採点。
            if self.cfg.global_match {
                self.match_reseed(&beams);
            }
            return;
        }
        // ロスト中の観測モデル (lost_sigma_per_m > 0 で有効)。ビームごとに
        // 距離 256 段 → 尤度の LUT を張る (exp はビーム数 × 256 回だけ —
        // セルあたりの評価コストは固定 σ の lf 参照と同じに保つ)。2 つを焼き込む:
        // (1) 距離比例 σ: σ_eff = σ0 + r·lost_sigma_per_m (config doc 参照)。
        // (2) スキャン内テンパリング: z_min ミキシング済みビーム尤度の (1/M) 乗。
        //     積が幾何平均になり、1 観測のダイナミックレンジが [z_min, 1] に
        //     圧縮される。生の積は「ビームあたり 25% 劣るだけ」の仮説を 2 観測
        //     で相対枝刈り (1e-4) の下へ落とし、リセット直後の真位置仮説が
        //     偶然の最良適合セルに 0.4 m で永久に殺される (津田沼 K4 で実測 —
        //     リセット 2 観測後に真値の重みが 0)。スキャン間の相関ゲート
        //     (lost_update_min_d/a) と同じ「相関証拠を独立扱いで過大計上しない」
        //     をスキャン内の 18 ビームにも適用した形。
        let lut: Option<Vec<[f32; 256]>> =
            (lost && self.cfg.lost_sigma_per_m > 0.0).then(|| self.lost_lut(&beams, 0.0));
        let quality = if self.cfg.viterbi {
            self.vit_observe(&beams, lut.as_deref())
        } else {
            self.sum_observe(&beams, lut.as_deref())
        };
        self.quality = quality;
        self.q_ewma = (1.0 - EWMA_BETA) * self.q_ewma + EWMA_BETA * quality;
        if self.active.is_empty() {
            // 尤度が全セルでアンダーフローする等の全滅 — 一様から出直す
            // (global_match はスキャンマッチから)。
            if self.cfg.global_match {
                self.match_reseed(&beams);
            } else {
                self.enter_uniform_free();
            }
            return;
        }
        // ラッチ条件は mix_uniform の**前**に確定させる — リセットは q_ewma を
        // reset_quality へ書き戻すので、後から見ると不一致の証拠が消えている。
        let mismatched = self.q_ewma < self.cfg.reset_quality;
        // 既にロストで belief がまだ広い (ESS > lost_ess) 間は再ミックスしない —
        // 一様混合直後の加重平均 quality は「ゴミの平均」で恒常的に低く、毎観測
        // リセットすると濃縮が永遠に始まらない上、active が毎回全域に戻って
        // 観測コストも爆発する (E 正規化で顕在化 — 津田沼 K4 で実測: 真値が
        // 850 tick 一様床に凍結、observe 平均 645 ms)。濃縮が進んで ESS が
        // lost_ess を割れば quality は意味を取り戻し、そのときの不一致は従来
        // どおり再拡散する (EMCL の expansion resetting の本来の対象)。
        if mismatched && !(self.lost && self.ess_c > self.cfg.lost_ess) {
            if self.lost {
                // ロスト中の再リセットは mix でなく**全平坦化**。mix は誤ピーク
                // (ゴースト) に質量 (1-α) を残したまま一様床を 1/(n·α) で張る
                // ので、床上の真の仮説は相対枝刈り (max × weight_skip_ratio) の
                // 3 桁下から始まり、確立する前に必ず刈られる (津田沼 K4 で
                // 実測: 検出時のゴースト 0.3 vs 床 1.1e-8 — 粒子フィルタの
                // 注入粒子はリサンプリングで対等になるが、ヒストグラム +
                // 相対枝刈りでは桁が合わない)。未ロストの最初の mix は残す —
                // 誤警報 (追跡中の一時的な不一致) なら生き残ったピークが
                // 次の観測で回復する。global_match は平坦化の代わりに
                // このスキャン (別視点) で候補を引き直す — リトライの本体。
                // ただし [`MATCH_RETRY_M`] のレート制限つき: 並進が足りない
                // うちは belief に触らず観測の淘汰に任せる (毎不一致の再マッチ
                // は反証済みエイリアスを復活させ、真値の蓄積優位を壊す)。
                if self.cfg.global_match {
                    if self.move_since_match >= MATCH_RETRY_M {
                        self.match_reseed(&beams);
                        return;
                    }
                } else {
                    self.enter_uniform_free();
                }
            } else if self.cfg.global_match {
                // 追跡からの最初の不一致 (一時破綻 or 誘拐検出) も informed に:
                // mix_uniform は free 一様 30M セルを flood し (次の observe が
                // 最大 ~14 s)、候補集合の淘汰の記憶も流してしまう — マッチ候補の
                // 等化混合に載せ替える (EMCL の expansion resetting の提案分布を
                // スキャンマッチにした形)。注入は**比較採用**: 推定近傍
                // ([`LOCAL_REMATCH_R_M`]) だけの候補と全域の候補を同じ LUT で
                // 採点し、近傍最良が全域最良に見劣りしなければ近傍だけを注入
                // する。低特徴区間の追跡の滑り (津田沼 K4 ルートで誘拐なしでも
                // 起きる一時破綻) では真値近傍がスキャンを全域と同格に説明する
                // ので、遠方エイリアスの注入 (復帰サイクル churn の源) を避け
                // られる。本物の誘拐では旧位置近傍が全域に大きく見劣りして
                // 従来どおり全域再シードに落ちる ([`LOCAL_REMATCH_ACCEPT`] の
                // doc — 絶対しきい値は誤アンカーを保護して逆効果)。レート制限は
                // どちらの注入にもかかる (毎不一致の再注入は淘汰の記憶を壊す)。
                if self.move_since_match >= MATCH_RETRY_M {
                    if let Some(p) = self.mean() {
                        let local = self.match_candidates(&beams, Some((p.x, p.y)));
                        let global = self.match_candidates(&beams, None);
                        let lb = local.iter().map(|c| c.0).fold(0.0f32, f32::max);
                        let gb = global.iter().map(|c| c.0).fold(0.0f32, f32::max);
                        let take_local = lb > 0.0 && lb >= LOCAL_REMATCH_ACCEPT * gb;
                        eprintln!(
                            "belief: rematch {} from tracking @ ({:.1}, {:.1}) — local \
                             {lb:.2} vs global {gb:.2}",
                            if take_local { "local" } else { "global" },
                            p.x,
                            p.y,
                        );
                        if take_local {
                            self.reseed_with(&local);
                        } else {
                            self.reseed_with(&global);
                        }
                    } else {
                        self.match_reseed(&beams);
                    }
                    return;
                }
            } else {
                self.mix_uniform();
            }
        }
        self.recompute_ess();
        if mismatched || self.ess_c > self.cfg.lost_ess {
            self.lost = true;
            self.probation = 0;
            self.rel_hold = 0;
            self.rel_anchor = None;
        } else if self.lost && self.try_release() {
            // 解除は**単峰性のみ**で判定する (多峰のままなら lost を維持して
            // 能動的再定位に判別させる)。かつては ess < contract_ess も要求して
            // いたが、E 正規化 + テンパリングの軟化した観測モデルでは勝者の
            // 質量シェアが ~1% で頭打ちし、ESS は数千のまま二度と 50 を割らない
            // (津田沼 K4 で実測: 真値が 127 s 間 argmax なのに解除されず
            // タイムアウト)。固定 ESS しきい値は地図スケールにも追従しない。
            // global_match はさらに**ピーク保持** ([`RELEASE_HOLD_OBS`]) を課す
            // (try_release doc 参照)。誤単峰への早すぎる解除は probation
            // (probation_obs) が受け持つ — 解除を安くして検証を走行に置くのが
            // 分業。
            self.lost = false;
            if self.cfg.probation_obs > 0 {
                // 仮解除 — 検証走行の予測整合を見てから確定する。解除を
                // 決めた静止観測そのものは検証に数えない (else if で分離)。
                self.probation = self.cfg.probation_obs;
                self.prob_move = 0.0;
            }
        } else if self.probation > 0 {
            // probation: lost_update_min_d_m の並進ごとに瞬時 quality を検査。
            // EWMA でなく瞬時なのは、廊下エイリアスの破綻が「開口部を通過した
            // 1 観測」で現れるため — 平均は正しい区間に薄められる。
            self.prob_move += moved_m;
            if self.prob_move >= self.cfg.lost_update_min_d_m {
                self.prob_move = 0.0;
                if self.quality < self.cfg.probation_min_q {
                    self.fail_probation();
                } else {
                    self.probation -= 1;
                }
            }
        }
    }

    /// probation 中か (仮解除 — pose は返るが確定前)。
    pub fn in_probation(&self) -> bool {
        self.probation > 0
    }

    /// probation を失敗させて lost へ戻す。observe 内の quality 破綻のほか、
    /// 呼び出し側が「誤姿勢で方策が引けない」等の外部証拠で落とすのにも使う。
    /// 半量 mix ([`Belief::mix_uniform`]) でないのは、mix 後も ESS が contract
    /// を割ったままで解除↔失敗が観測 1 回ごとに振動するため — 全域一様へ
    /// 戻して出直す (誤った証拠は捨てるのが正しい)。global_match はスキャンを
    /// 持たないここでは空にするだけ — 次の observe の全滅復旧経路が、その
    /// スキャンでマッチ再シードする。
    pub fn fail_probation(&mut self) {
        if self.cfg.global_match {
            self.enter_lost_empty();
        } else {
            self.enter_uniform_free();
        }
    }

    /// レーザーが貫通したセルを free 扱いにして、地図の壁を反証する
    /// (`ValueIteratorLocal::clear_map_from_scan` の belief 側)。
    ///
    /// VI 側だけ開けても belief は動けない — 予測 (shift/diffuse) は非 free へ
    /// 落ちた質量を捨てるので、幽霊壁の向こうへ実機が進んでも推定姿勢は壁の
    /// 手前に張り付いたままになる。free マスクは VI の `states[].free` とは
    /// 別のコピーなので、同じ反証をこちらにも当てる必要がある。尤度場は
    /// 変えないので、開いた場所の重みは地図基準のまま低い。
    /// 寿命は VI 側と違って永続 — 地図が間違っているならずっと間違っている。
    pub fn clear_free_from_scan(&mut self, scan: &LaserScan) {
        let Some(p) = self.mean() else { return };
        let step = self.res.min(self.field.res) * 0.5;
        let (nx, ny, ox, oy, res) = (self.nx, self.ny, self.ox, self.oy, self.res);
        let (free, field) = (&mut self.free, &mut self.field);
        let mut opened = 0usize;
        crate::local::walk_beams(scan, p.x, p.y, p.yaw_rad, step, res, |wx, wy| {
            let ix = ((wx - ox) / res).floor() as i32;
            let iy = ((wy - oy) / res).floor() as i32;
            if ix < 0 || iy < 0 || ix >= nx || iy >= ny {
                return false; // 地図外 — ビームは直進するので戻ってこない
            }
            let i = (iy * nx + ix) as usize;
            if !free[i] {
                free[i] = true;
                opened += 1;
            }
            field.mark_free(wx, wy);
            true
        });
        self.n_free += opened;
    }

    /// 現在の推定姿勢。ロスト ([`Belief::is_lost`]) と未初期化は None。free
    /// スナップ (壁・未知に落ちた平均の実在仮説への吸着) は旧実装のまま。
    pub fn pose(&self) -> Option<PoseView> {
        if !self.initialized || self.is_lost() {
            return None;
        }
        let m = self.mean()?;
        let m = if self.field.free_at(m.x, m.y) {
            m
        } else {
            // 多峰・ドーナツ状 belief の平均は穴 (壁・未知) に落ちる — 実在する
            // 仮説 (free 上の mode) へ吸着して返す。
            self.mode_free().unwrap_or(m)
        };
        // 残 pend (未 flush の移動) の解析加算 — flush は observe 冒頭なので、
        // 観測間の姿勢は平均へ足すだけで済む (広がりの増分は次の flush が持つ)。
        Some(PoseView {
            x: m.x + self.pend_f * m.yaw_rad.cos(),
            y: m.y + self.pend_f * m.yaw_rad.sin(),
            yaw_rad: m.yaw_rad + self.pend_rot_deg.to_radians(),
        })
    }

    /// 診断: 姿勢の周辺 (±1 セル・±1 θ ビン) の最大重みと、belief 全体の最大
    /// 重みとその位置。ベンチが真値仮説の生死と「誰に負けたか」を追う用 —
    /// 相対枝刈りで 0 に落ちた仮説は以後の観測では復活できない。
    pub fn probe_weight(&self, p: PoseView) -> (f64, f64, Option<PoseView>) {
        if !self.initialized {
            return (0.0, 0.0, None);
        }
        let cx = ((p.x - self.ox) / self.res).floor() as i32;
        let cy = ((p.y - self.oy) / self.res).floor() as i32;
        let deg = p.yaw_rad.to_degrees().rem_euclid(360.0);
        let ct = ((deg / self.t_res_deg) as i32).clamp(0, self.nt - 1);
        let mut w = 0.0f32;
        for dt in -1..=1 {
            let it = (ct + dt + self.nt) % self.nt;
            for dy in -1..=1 {
                for dx in -1..=1 {
                    let (ix, iy) = (cx + dx, cy + dy);
                    if ix < 0 || iy < 0 || ix >= self.nx || iy >= self.ny {
                        continue;
                    }
                    w = w.max(self.b[bidx2(self.nx, self.ny, ix, iy, it)]);
                }
            }
        }
        let mut maxw = 0.0f32;
        let mut arg = None;
        for &i in &self.active {
            if self.b[i as usize] > maxw {
                maxw = self.b[i as usize];
                arg = Some(i);
            }
        }
        let win = arg.map(|i| {
            let (ix, iy, it) = self.decode(i);
            let (x, y) = self.cell_center(ix, iy);
            PoseView { x, y, yaw_rad: self.theta_center(it) }
        });
        (w as f64, maxw as f64, win)
    }

    /// belief の上位 `k` 仮説 (セル中心の姿勢, 正規化重み)。重み降順 — QMDP
    /// の仮説集合の契約。ロスト中も多峰のまま返す。
    pub fn top_cells(&self, k: usize) -> Vec<(PoseView, f64)> {
        if !self.initialized || k == 0 {
            return Vec::new();
        }
        let mut cells: Vec<(f32, u32)> = self
            .active
            .iter()
            .filter(|&&i| self.b[i as usize] > 0.0)
            .map(|&i| (self.b[i as usize], i))
            .collect();
        if cells.len() > k {
            cells.select_nth_unstable_by(k - 1, |a, b| b.0.total_cmp(&a.0));
            cells.truncate(k);
        }
        cells.sort_by(|a, b| b.0.total_cmp(&a.0));
        cells
            .into_iter()
            .map(|(w, i)| {
                let (ix, iy, it) = self.decode(i);
                let (x, y) = self.cell_center(ix, iy);
                (PoseView { x, y, yaw_rad: self.theta_center(it) }, w as f64)
            })
            .collect()
    }

    /// ロスト中の能動的再定位の行き先候補 ([`crate::localize::Localizer::reloc_targets`]
    /// と同じ契約)。
    /// 非ロスト = 空 — 走って判別する価値があるのは pose を止めている間だけ。
    ///
    /// 峰の取り方だけが窓つき推定器と違う: 全地図 belief は 1 つの峰でも数千セル
    /// あるので `top_cells(k)` の上位 k では 2 つ目の峰に届かない。最大重みの
    /// [`MODE_THRESHOLD`] 以上のセル全部から [`modes`] で間引く。
    ///
    /// ロスト中は毎 tick 呼ばれるので、並べるのは候補が [`MODE_CANDIDATE_CAP`]
    /// 以下のときだけ (走査は active の線形 2 パスで確保)。それを超える = belief
    /// が一様に近い ⇒ 判別すべき峰が無い ⇒ 空 = 受動復帰。
    pub fn reloc_targets(&self) -> Vec<(f64, f64)> {
        if !self.initialized || !self.lost {
            return Vec::new();
        }
        let Some(hyps) = self.strong_hyps() else {
            return Vec::new();
        };
        let mut m = modes(&hyps, MODE_MIN_SEP_M);
        m.truncate(RELOC_MODES);
        reloc_targets(
            &m,
            |x, y| self.field.free_at(x, y),
            |x, y| self.field.at(x, y),
            self.cfg.reloc_scale,
        )
    }

    /// 最大重みの [`MODE_THRESHOLD`] 倍以上のセルを重み降順の仮説列に。候補が
    /// [`MODE_CANDIDATE_CAP`] 超 (= belief が一様に近く峰がまだ無い) と全滅は
    /// None。[`Belief::reloc_targets`] と [`Belief::release_unimodal`] の共通部。
    fn strong_hyps(&self) -> Option<Vec<(PoseView, f64)>> {
        let maxw = self.active.iter().fold(0.0f32, |m, &i| m.max(self.b[i as usize]));
        if maxw <= 0.0 {
            return None;
        }
        let thr = maxw * MODE_THRESHOLD;
        let n_cand = self.active.iter().filter(|&&i| self.b[i as usize] >= thr).count();
        if n_cand > MODE_CANDIDATE_CAP {
            return None;
        }
        let mut cells: Vec<(f32, u32)> = self
            .active
            .iter()
            .filter(|&&i| self.b[i as usize] >= thr)
            .map(|&i| (self.b[i as usize], i))
            .collect();
        cells.sort_by(|a, b| b.0.total_cmp(&a.0));
        Some(
            cells
                .into_iter()
                .map(|(w, i)| {
                    let (ix, iy, it) = self.decode(i);
                    let (x, y) = self.cell_center(ix, iy);
                    (PoseView { x, y, yaw_rad: self.theta_center(it) }, w as f64)
                })
                .collect(),
        )
    }

    /// 解除の単峰判定。flatten 経路は従来どおり top-64 セルの mode_count。
    /// global_match は**相対峰**判定: 最大重みの 5% 以上の峰が 1 つだけなら
    /// 解除。絶対量 (top-64 の質量シェアや ESS) はテンパリング床の広い裾で
    /// セルあたり重みが ~0.7% で頭打ちして成立しない (津田沼 K1 で実測: 真値が
    /// argmax ratio 0.99 のまま解除されず凍結)。相対峰なら「真値が max の 10%
    /// で生存中の誤単峰」(旧 top-64 判定の穴) も競合峰として塞がる。
    fn release_unimodal(&self) -> bool {
        if !self.cfg.global_match {
            return mode_count(&self.top_cells(UNIMODAL_TOP_K), MODE_MIN_SEP_M) <= 1;
        }
        match self.strong_hyps() {
            Some(h) => mode_count(&h, MODE_MIN_SEP_M) <= 1,
            None => false,
        }
    }

    /// 解除判定の入口。単峰 ([`Belief::release_unimodal`]) に加え、global_match
    /// では**ピーク保持** ([`RELEASE_HOLD_OBS`]) を課す: argmax ピークが
    /// [`MODE_MIN_SEP_M`] 以内に留まる観測が連続するまで解除しない。誤ピーク
    /// の勝者は再マッチ・淘汰のたびにテレポートするのに対し、真値ピークは
    /// 何百 tick も動かない (津田沼で実測: ratio 0.99 が 175 tick) — 「勝者の
    /// 持続」が真偽を分ける安価な信号で、これが無いと一瞬の誤単峰への解除 →
    /// probation の誤走行 → 棄却のサイクルが走行予算を吸収する。
    fn try_release(&mut self) -> bool {
        if !self.release_unimodal() {
            self.rel_hold = 0;
            return false;
        }
        if !self.cfg.global_match {
            return true;
        }
        let Some((pk, _)) = self.strong_hyps().and_then(|h| h.first().copied()) else {
            self.rel_hold = 0;
            return false;
        };
        let held = self
            .rel_anchor
            .is_some_and(|(ax, ay)| (pk.x - ax).hypot(pk.y - ay) <= MODE_MIN_SEP_M);
        self.rel_hold = if held { self.rel_hold + 1 } else { 1 };
        self.rel_anchor = Some((pk.x, pk.y));
        if self.rel_hold >= RELEASE_HOLD_OBS {
            self.rel_hold = 0;
            true
        } else {
            false
        }
    }

    /// belief の θ 周辺分布を可視化用 OccupancyGrid に描く (未シードなら None)。
    /// 格子は VI と同一 = `value_function` と重ねて見られる。active だけ舐める
    /// ので、収束後は数百セルぶんの仕事しかしない。
    pub fn grid(&self) -> Option<OccupancyGrid> {
        if !self.initialized {
            return None;
        }
        let plane = (self.nx as usize) * (self.ny as usize);
        let mut m = vec![0f32; plane];
        for &i in &self.active {
            m[i as usize % plane] += self.b[i as usize];
        }
        Some(mass_to_grid(&m, self.nx, self.ny, self.res, self.ox, self.oy, self.oq.clone()))
    }

    // ═══ 内部: 共通機構 ═══

    /// 重み付き平均 (θ は円環平均)。合計 0 なら None。active だけ舐める。
    fn mean(&self) -> Option<PoseView> {
        let (mut sw, mut sx, mut sy, mut sc, mut ss) = (0.0f64, 0.0, 0.0, 0.0, 0.0);
        for &iu in &self.active {
            let w = self.b[iu as usize] as f64;
            if w <= 0.0 {
                continue;
            }
            let (ix, iy, it) = self.decode(iu);
            let th = self.theta_center(it);
            let (cx, cy) = self.cell_center(ix, iy);
            sw += w;
            sx += w * cx;
            sy += w * cy;
            sc += w * th.cos();
            ss += w * th.sin();
        }
        (sw > 0.0).then(|| PoseView { x: sx / sw, y: sy / sw, yaw_rad: ss.atan2(sc) })
    }

    /// free セル上の最大重み仮説 (mode)。free 上に質量が無ければ None。
    fn mode_free(&self) -> Option<PoseView> {
        let mut best: Option<(f32, u32)> = None;
        for &iu in &self.active {
            let w = self.b[iu as usize];
            if w > 0.0 && best.map_or(true, |(bw, _)| w > bw) {
                let (ix, iy, _) = self.decode(iu);
                if self.free[(iy * self.nx + ix) as usize] {
                    best = Some((w, iu));
                }
            }
        }
        best.map(|(_, iu)| {
            let (ix, iy, it) = self.decode(iu);
            let (x, y) = self.cell_center(ix, iy);
            PoseView { x, y, yaw_rad: self.theta_center(it) }
        })
    }

    /// active 上の正規化。
    fn normalize_active(&mut self) {
        let sum: f64 = self.active.iter().map(|&i| self.b[i as usize] as f64).sum();
        if sum > 0.0 && sum.is_finite() {
            // ミスマッチ時のビーム積は f32 subnormal 域まで沈む。そこで
            // `(1/sum) as f32` を掛けると inv が ∞ に飽和して 0×∞ = NaN が belief に
            // 混ざり、EWMA が NaN → リセットが二度と発火しなくなる。f64 で割る。
            for &i in &self.active {
                let v = &mut self.b[i as usize];
                *v = ((*v as f64) / sum) as f32;
            }
        } else if !sum.is_finite() {
            // ∞/NaN が混ざったら復旧不能 — 0 へ落とし、全滅復旧
            // (enter_uniform_free) に回復を任せる。
            for &i in &self.active {
                self.b[i as usize] = 0.0;
            }
            self.active.clear();
        }
    }

    /// ESS キャッシュの更新 (b が変わる操作の末尾で呼ぶ)。
    fn recompute_ess(&mut self) {
        let s2: f64 = self
            .active
            .iter()
            .map(|&i| {
                let v = self.b[i as usize] as f64;
                v * v
            })
            .sum();
        self.ess_c = if s2 > 0.0 { 1.0 / s2 } else { 0.0 };
    }

    /// 溜めた移動量の一括適用 (observe 冒頭)。sum-product はアクティブセルの
    /// scatter シフト + 拡散、min-plus は整数シフト + 緩和。適用後 pend は 0。
    fn flush(&mut self) {
        let (pf, pt_deg, ticks) = (self.pend_f, self.pend_rot_deg, self.pend_ticks);
        self.pend_f = 0.0;
        self.pend_rot_deg = 0.0;
        self.pend_ticks = 0;
        if self.cfg.viterbi {
            let n = self.b.len();
            let (nx, ny, nt) = (self.nx, self.ny, self.nt);
            // セル未満の移動は繰り越さず捨てる — 半セルの誤差は緩和が吸収する。
            if pf.abs() >= 0.5 * self.res || pt_deg.abs() >= 0.5 * self.t_res_deg {
                minplus_shift(
                    &mut self.delta[..n],
                    &mut self.scratch[..n],
                    nx,
                    ny,
                    nt,
                    self.res,
                    self.t_res_deg,
                    pf,
                    pt_deg,
                );
            }
            // 移動ゼロでも回す (sum-product 側が毎 tick 拡散するのと同役)。
            // scratch の全ゼロ不変は sum-product 経路専用なので汚してよい。
            minplus_relax(&mut self.delta[..n], nx, ny, nt, VIT_LAMBDA_XY, VIT_LAMBDA_T);
            return;
        }
        if ticks == 0 && pf == 0.0 && pt_deg == 0.0 {
            return;
        }
        if pf != 0.0 || pt_deg != 0.0 {
            self.shift_scatter(pf, pt_deg);
        }
        let a_xy = blur_a(self.cfg.motion_sigma_xy_m / self.res) * ticks as f64;
        let a_t = if self.nt > 2 {
            blur_a(self.cfg.motion_sigma_theta_deg / self.t_res_deg) * ticks as f64
        } else {
            0.0
        };
        if a_xy > 0.0 || a_t > 0.0 {
            // 6 近傍 scatter の中心重み 1-4a_xy-2a_t を余裕をもって非負に保つ
            // pass 分割 (旧 blur_a の 0.25 クランプの後継 — 1 pass で表せない
            // 累積拡散は複数 pass で表す)。
            let passes = ((4.0 * a_xy + 2.0 * a_t) / 0.5).ceil().max(1.0) as usize;
            let (pa_xy, pa_t) = ((a_xy / passes as f64) as f32, (a_t / passes as f64) as f32);
            for _ in 0..passes {
                self.diffuse_scatter(pa_xy, pa_t);
            }
        }
        // シフト・拡散が壁・地図外へ落とした質量を回収 (物理拘束後の再正規化)。
        self.normalize_active();
        self.recompute_ess();
    }

    /// sum-product のシフト: 各アクティブセルを、その θ の世界方向へ回した
    /// 前進 pf + θ 回転ぶんだけ trilinear scatter で動かす (旧 predict の
    /// 後方双線形サンプリングの、疎集合向け前方版)。非 free・地図外に落ちる
    /// 質量は捨てる。
    fn shift_scatter(&mut self, pf: f64, pt_deg: f64) {
        let (nx, ny, nt) = (self.nx, self.ny, self.nt);
        let ft = pt_deg / self.t_res_deg;
        let mut cand: Vec<u32> = Vec::with_capacity(self.active.len().saturating_mul(8));
        let active = std::mem::take(&mut self.active);
        for &iu in &active {
            let i = iu as usize;
            let w = self.b[i];
            // 旧側を掃除しておく (swap 後に scratch となる — 全ゼロ不変の維持)。
            self.b[i] = 0.0;
            if w <= 0.0 {
                continue;
            }
            let (ix, iy, it) = self.decode(iu);
            let th = self.theta_center(it);
            let ux = ix as f64 + pf * th.cos() / self.res;
            let uy = iy as f64 + pf * th.sin() / self.res;
            let ut = it as f64 + ft;
            let (x0, y0, t0) = (ux.floor(), uy.floor(), ut.floor());
            let (fx, fy, ftr) = (ux - x0, uy - y0, ut - t0);
            for (ot, wt) in [(0i32, 1.0 - ftr), (1, ftr)] {
                if wt <= 0.0 {
                    continue;
                }
                let jt = (t0 as i32 + ot).rem_euclid(nt);
                for (oy, wy) in [(0i32, 1.0 - fy), (1, fy)] {
                    let jy = y0 as i32 + oy;
                    if wy <= 0.0 || jy < 0 || jy >= ny {
                        continue;
                    }
                    for (ox, wx) in [(0i32, 1.0 - fx), (1, fx)] {
                        let jx = x0 as i32 + ox;
                        if wx <= 0.0 || jx < 0 || jx >= nx {
                            continue;
                        }
                        if !self.free[(jy * nx + jx) as usize] {
                            continue;
                        }
                        let j = bidx2(nx, ny, jx, jy, jt);
                        deposit(&mut self.scratch, &mut cand, j, w * (wt * wy * wx) as f32);
                    }
                }
            }
        }
        std::mem::swap(&mut self.b, &mut self.scratch);
        self.active = cand;
    }

    /// sum-product の拡散 1 pass: 6 近傍 (±x, ±y, ±θ) への scatter。旧 blur の
    /// 3 点カーネルを軸ごとに掛ける代わりに、疎集合の 1 回の scatter で近似する
    /// (pass 分割は flush 側)。非 free へ漏れた質量は捨てる (物理拘束)。
    fn diffuse_scatter(&mut self, a_xy: f32, a_t: f32) {
        let (nx, ny, nt) = (self.nx, self.ny, self.nt);
        let wc = 1.0 - 4.0 * a_xy - 2.0 * a_t; // pass 分割が非負を保証
        let mut cand: Vec<u32> = Vec::with_capacity(self.active.len().saturating_mul(7));
        let active = std::mem::take(&mut self.active);
        for &iu in &active {
            let i = iu as usize;
            let w = self.b[i];
            self.b[i] = 0.0;
            if w <= 0.0 {
                continue;
            }
            let (ix, iy, it) = self.decode(iu);
            // 中心 (発生元は free)。
            deposit(&mut self.scratch, &mut cand, i, w * wc);
            for (jx, jy) in [(ix - 1, iy), (ix + 1, iy), (ix, iy - 1), (ix, iy + 1)] {
                if jx < 0 || jx >= nx || jy < 0 || jy >= ny {
                    continue;
                }
                if !self.free[(jy * nx + jx) as usize] {
                    continue;
                }
                deposit(&mut self.scratch, &mut cand, bidx2(nx, ny, jx, jy, it), w * a_xy);
            }
            if a_t > 0.0 {
                for jt in [(it + nt - 1) % nt, (it + 1) % nt] {
                    deposit(&mut self.scratch, &mut cand, bidx2(nx, ny, ix, iy, jt), w * a_t);
                }
            }
        }
        std::mem::swap(&mut self.b, &mut self.scratch);
        self.active = cand;
    }

    /// sum-product の補正: アクティブセルの重みへビーム尤度の積を乗じ、乗算後の
    /// 相対しきい値で枝刈りして正規化する。戻り値は観測一致度。
    ///
    /// 枝刈りを乗算**後**に置くのは意図的 — 乗算前に相対しきい値で切ると、
    /// 一様混合リセットが張った床 (α/free 数 ≪ max·ratio) がビーム評価される
    /// 前に消え、リセットが機能しなくなるため。
    fn sum_observe(&mut self, beams: &[(f64, f64)], lut: Option<&[[f32; 256]]>) -> f64 {
        let z_min = self.cfg.z_min;
        let m_inv = 1.0 / beams.len() as f64;
        // 焼きなまし強度は observe 内で一定 (global_match のロスト経路のみ有効)。
        let apow = if self.cfg.global_match { self.anneal_pow() } else { 1 };
        // 追跡経路の未知端点クランプ / ロスト経路の証拠不足の頭打ち定数。
        let unk_l = z_min + (1.0 - z_min) * UNKNOWN_L;
        // ビーム角は加法定理で回す (セルごとに全ビームの sin/cos を呼ばない)。
        let bt: Vec<(f64, f64, f64)> =
            beams.iter().map(|&(ba, r)| (ba.cos(), ba.sin(), r)).collect();
        let mut quality = 0.0f64;
        let mut active = std::mem::take(&mut self.active);
        active.retain(|&iu| {
            let i = iu as usize;
            let w = self.b[i];
            if w <= 0.0 {
                return false;
            }
            let (ix, iy, it) = self.decode(iu);
            // 物理拘束: 壁・未知の中の仮説はビーム評価するまでもなく棄却。
            if !self.free[(iy * self.nx + ix) as usize] {
                self.b[i] = 0.0;
                return false;
            }
            let th = self.theta_center(it);
            let (ct, st) = (th.cos(), th.sin());
            let (cx, cy) = self.cell_center(ix, iy);
            let (mut prod, mut known) = (1.0f64, 0u32);
            for (bi, &(cb, sb, r)) in bt.iter().enumerate() {
                let (ca, sa) = (ct * cb - st * sb, st * cb + ct * sb);
                let (px, py) = (cx + r * ca, cy + r * sa);
                match lut {
                    // ロスト観測モデル: LUT は z_min ミキシング + (1/M)
                    // テンパリング焼き込み済み。未知・地図外の端点は分母から
                    // 除外 (E 正規化 — LOST_MIN_KNOWN の doc 参照)。
                    Some(t) => {
                        if let Some(d) = self.field.known_dist(px, py) {
                            prod *= t[bi][d] as f64;
                            known += 1;
                        }
                    }
                    None => {
                        let mut lb = z_min + (1.0 - z_min) * self.field.at(px, py);
                        // 未知セルの端点は中立 (UNKNOWN_L) との max。
                        if lb < unk_l && self.field.unk_at(px, py) {
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
                let bwv = if self.cfg.global_match {
                    (prod * unk_l.powf((bt.len() - known as usize) as f64 * m_inv))
                        .powf(apow as f64)
                } else {
                    s
                };
                (s, bwv)
            } else {
                (prod.powf(m_inv), prod)
            };
            quality += w as f64 * gm;
            self.b[i] = (w as f64 * bw) as f32;
            true
        });
        // 枝刈り: 乗算後の集中をアクティブ集合へ反映 (weight_skip_ratio が
        // 疎な作業集合のしきい値を兼ねる)。
        let maxw = active.iter().map(|&i| self.b[i as usize]).fold(0.0f32, f32::max);
        let thr = maxw * self.cfg.weight_skip_ratio as f32;
        active.retain(|&iu| {
            let i = iu as usize;
            if self.b[i] <= thr {
                self.b[i] = 0.0;
                false
            } else {
                true
            }
        });
        self.active = active;
        self.normalize_active();
        quality
    }

    /// EMCL 風リセット: free 一様を [`MIX_UNIFORM`] だけ混ぜる (旧 expand の
    /// 混合部だけの salvage — 粗レベルへの射影は無い)。min-plus では free 全
    /// セルへ δ の床を与える等価操作。
    fn mix_uniform(&mut self) {
        if self.n_free == 0 {
            return;
        }
        if self.cfg.viterbi {
            let lam = self.reset_floor_ln() as f32;
            let dmin = self
                .active
                .iter()
                .map(|&i| self.delta[i as usize])
                .fold(f32::INFINITY, f32::min);
            if !dmin.is_finite() {
                // 全滅は observe / vit_observe 側の自己修復に任せる。
                return;
            }
            self.active.clear();
            for it in 0..self.nt {
                for iy in 0..self.ny {
                    let row = (iy * self.nx) as usize;
                    for ix in 0..self.nx {
                        if !self.free[row + ix as usize] {
                            continue;
                        }
                        let i = bidx2(self.nx, self.ny, ix, iy, it);
                        let d = self.delta[i].min(dmin + lam);
                        self.delta[i] = d;
                        // b = exp(δmin − δ) の再実体化 (床込み)。
                        self.b[i] = ((dmin - d) as f64).exp() as f32;
                        self.active.push(i as u32);
                    }
                }
            }
            self.normalize_active();
        } else {
            // (1−α)·belief + α·free-一様。belief は正規化済みなので総和は 1 のまま。
            let alpha = MIX_UNIFORM;
            for &i in &self.active {
                self.b[i as usize] *= 1.0 - alpha;
            }
            let u = alpha / (self.n_free * self.nt as usize) as f32;
            self.active.clear();
            for it in 0..self.nt {
                for iy in 0..self.ny {
                    let row = (iy * self.nx) as usize;
                    for ix in 0..self.nx {
                        if !self.free[row + ix as usize] {
                            continue;
                        }
                        let i = bidx2(self.nx, self.ny, ix, iy, it);
                        self.b[i] += u;
                        self.active.push(i as u32);
                    }
                }
            }
        }
        // リセット直後に再リセットしない。
        self.q_ewma = self.cfg.reset_quality;
    }

    // ═══ 内部: 全域マッチャ (global_match) ═══

    /// ロスト観測モデルのビーム別 LUT (距離インデックス 256 段 → 尤度)。
    /// 距離比例 σ ([`BeliefConfig::lost_sigma_per_m`]) とスキャン内テンパリング
    /// (1/M 乗) を焼き込む — observe のロスト経路 (observe 内コメント参照) と
    /// [`Belief::match_reseed`] が共有。`extra_sigma_m` はマッチャの粗段が
    /// ブロック中心 1 点でブロック内全姿勢を代表するための σ 繰り込み
    /// (端点はブロック半径ぶん振れる — 距離比例 σ と同じ発想)。
    fn lost_lut(&self, beams: &[(f64, f64)], extra_sigma_m: f64) -> Vec<[f32; 256]> {
        let res = self.field.res;
        let z_min = self.cfg.z_min;
        let m_inv = 1.0 / beams.len() as f64;
        beams
            .iter()
            .map(|&(_, r)| {
                let s = self.cfg.sensor_sigma_m + self.cfg.lost_sigma_per_m * r + extra_sigma_m;
                let inv_2s2 = 1.0 / (2.0 * s * s);
                std::array::from_fn(|d| {
                    let dm = d as f64 * res;
                    let l = (-dm * dm * inv_2s2).exp();
                    (z_min + (1.0 - z_min) * l).powf(m_inv) as f32
                })
            })
            .collect()
    }

    /// 焼きなましの現在の証拠強度 (1..=[`LOST_EVIDENCE_POW`]) —
    /// 前回マッチからの走行距離で硬化する ([`MATCH_ANNEAL_M`])。
    #[inline]
    fn anneal_pow(&self) -> i32 {
        (1 + (self.move_since_match / MATCH_ANNEAL_M) as i32).clamp(1, LOST_EVIDENCE_POW)
    }

    /// [`MatchPool`] の構築 (初回マッチで 1 度だけ): free セルを含む belief
    /// ブロックの一覧。
    fn build_match_pool(&self) -> MatchPool {
        let s = MATCH_STRIDE_CELLS;
        let mut free_blocks = Vec::new();
        for by0 in (0..self.ny).step_by(s as usize) {
            for bx0 in (0..self.nx).step_by(s as usize) {
                'blk: for iy in by0..(by0 + s).min(self.ny) {
                    for ix in bx0..(bx0 + s).min(self.nx) {
                        if self.free[(iy * self.nx + ix) as usize] {
                            free_blocks.push((bx0, by0));
                            break 'blk;
                        }
                    }
                }
            }
        }
        MatchPool { free_blocks }
    }

    /// 全域相関スキャンマッチによる再シード — ロスト中の flatten
    /// (enter_uniform_free) の置き換え ([`BeliefConfig::global_match`])。
    ///
    /// 粗段: free ブロック中心 × 全 θ を「σ をブロック半径ぶん膨らませた」
    /// E 正規化ロストモデルで採点し、上位 [`MATCH_BRANCHES`] 枝を残す
    /// (correlative matching の粗レベル標準 — min-pool 上界は密集域で同点
    /// プラトーになり弁別しない)。詳細段: 各生存枝のブロック内 free セルを
    /// 真の σ で採点し、**枝ごとの勝者 1 セル**に 3×3×3 の塊を張る (全体
    /// top-K セルはプラトー 1 領域が席を独占して真値が落ちる — 空間多様性は
    /// 枝の粒度で保証する)。重み ∝ スコア — テンパリング済みなので 1 スキャン
    /// で過剰に確信しない。張り方は全とっかえでなく既存 belief との**等化
    /// 混合** (下のコメント参照) で、呼び出しは [`MATCH_RETRY_M`] でレート
    /// 制限される — どちらも生存仮説の淘汰の記憶を守るため。lost ラッチは維持 — 解除は
    /// 従来どおり単峰性 (+ 質量シェア) + probation の分業で、候補が全部外れ
    /// なら quality 破綻 → 次の (別視点の) スキャンで再マッチ。
    fn match_reseed(&mut self, beams: &[(f64, f64)]) {
        let cands = self.match_candidates(beams, None);
        self.reseed_with(&cands);
    }

    /// マッチ候補の計算 (belief には触らない共有部)。`window = Some((x, y))`
    /// で探索をその近傍 [`LOCAL_REMATCH_R_M`] のブロックに絞る — 追跡からの
    /// 一時破綻 (誘拐でない滑り) を、全域マッチの遠方エイリアス注入なしで
    /// 引き直すローカル再マッチ用。
    fn match_candidates(
        &mut self,
        beams: &[(f64, f64)],
        window: Option<(f64, f64)>,
    ) -> Vec<(f32, u32)> {
        if self.match_pool.is_none() {
            self.match_pool = Some(self.build_match_pool());
        }
        let s = MATCH_STRIDE_CELLS;
        // ブロック中心代表の σ 繰り込み = ブロックの半対角 [m]。
        let block_r = s as f64 * self.res * std::f64::consts::FRAC_1_SQRT_2;
        let lut_c = self.lost_lut(beams, block_r);
        let lut_f = self.lost_lut(beams, 0.0);
        // ── 候補計算 (共有借用のみ) ──
        let cands: Vec<(f32, u32)> = {
            let pool = self.match_pool.as_ref().unwrap();
            let f = &self.field;
            let (nt, m) = (self.nt, beams.len());
            // θ ビン × ビームの端点オフセットを前計算 (粗・詳細で共有)。
            let mut offs = Vec::with_capacity(nt as usize * m);
            for it in 0..nt {
                let th = self.theta_center(it);
                for &(ba, r) in beams {
                    let a = th + ba;
                    offs.push((r * a.cos(), r * a.sin()));
                }
            }
            let unk_l = (self.cfg.z_min + (1.0 - self.cfg.z_min) * UNKNOWN_L) as f32;
            // observe のロスト経路と同じ E 正規化採点 (LUT だけ粗・詳細で違う)。
            let score = |wx: f64, wy: f64, ob: usize, lut: &[[f32; 256]]| -> f32 {
                let (mut prod, mut known) = (1.0f64, 0u32);
                for (bi2, lt) in lut.iter().enumerate() {
                    let (ox2, oy2) = offs[ob + bi2];
                    if let Some(d) = f.known_dist(wx + ox2, wy + oy2) {
                        prod *= lt[d] as f64;
                        known += 1;
                    }
                }
                if known >= LOST_MIN_KNOWN {
                    prod.powf(m as f64 / known as f64) as f32
                } else {
                    unk_l
                }
            };
            let mut coarse: Vec<(f32, u32)> =
                Vec::with_capacity(pool.free_blocks.len() * nt as usize);
            for (bi, &(bx0, by0)) in pool.free_blocks.iter().enumerate() {
                let cwx = self.ox + (bx0 as f64 + s as f64 * 0.5) * self.res;
                let cwy = self.oy + (by0 as f64 + s as f64 * 0.5) * self.res;
                if let Some((wx0, wy0)) = window {
                    if (cwx - wx0).hypot(cwy - wy0) > LOCAL_REMATCH_R_M + block_r {
                        continue;
                    }
                }
                for it in 0..nt {
                    let sc = score(cwx, cwy, it as usize * m, &lut_c);
                    coarse.push((sc, bi as u32 * nt as u32 + it as u32));
                }
            }
            if coarse.len() > MATCH_BRANCHES {
                coarse.select_nth_unstable_by(MATCH_BRANCHES - 1, |a, b| b.0.total_cmp(&a.0));
                coarse.truncate(MATCH_BRANCHES);
            }
            let mut fine: Vec<(f32, u32)> = Vec::with_capacity(coarse.len());
            for &(_, code) in &coarse {
                let (bi, it) = ((code / nt as u32) as usize, (code % nt as u32) as i32);
                let (bx0, by0) = pool.free_blocks[bi];
                let ob = it as usize * m;
                // 枝の勝者 1 セル (真の σ で採点し直す)。
                let mut best: Option<(f32, u32)> = None;
                for iy in by0..(by0 + s).min(self.ny) {
                    for ix in bx0..(bx0 + s).min(self.nx) {
                        if !self.free[(iy * self.nx + ix) as usize] {
                            continue;
                        }
                        let (cx, cy) = self.cell_center(ix, iy);
                        let sc = score(cx, cy, ob, &lut_f);
                        if best.map_or(true, |(b, _)| sc > b) {
                            best = Some((sc, bidx2(self.nx, self.ny, ix, iy, it) as u32));
                        }
                    }
                }
                if let Some(w) = best {
                    fine.push(w);
                }
            }
            fine
        };
        cands
    }

    /// 候補集合からの再シード (可変借用側): 全とっかえでなく既存 belief と
    /// 等化混合する — 生存仮説 (真値含む) の淘汰の記憶を保持しつつ、枯れた
    /// 領域へ候補を再注入する (粒子フィルタの注入リサンプリングのヒストグラム
    /// 版)。既存が空 (初期化・fail_probation 後) なら全量新規。
    fn reseed_with(&mut self, cands: &[(f32, u32)]) {
        let (nx, ny, nt) = (self.nx, self.ny, self.nt);
        let mut newc: Vec<u32> = Vec::with_capacity(cands.len() * 8);
        let mut best = 0.0f32;
        for &(sc, iu) in cands {
            best = best.max(sc);
            let (ix, iy, it) = self.decode(iu);
            // 3×3×3 の塊 (軸重み [0.5, 1, 0.5] の積) — shift/diffuse の
            // trilinear が動ける最小の広がり。scratch に組み立てる (全ゼロ
            // 不変を fold 時に復元)。
            for (dt, wt) in [(-1i32, 0.5f32), (0, 1.0), (1, 0.5)] {
                let jt = (it + dt + nt) % nt;
                for (dy, wy) in [(-1i32, 0.5f32), (0, 1.0), (1, 0.5)] {
                    let jy = iy + dy;
                    if jy < 0 || jy >= ny {
                        continue;
                    }
                    for (dx, wx) in [(-1i32, 0.5f32), (0, 1.0), (1, 0.5)] {
                        let jx = ix + dx;
                        if jx < 0 || jx >= nx || !self.free[(jy * nx + jx) as usize] {
                            continue;
                        }
                        deposit(
                            &mut self.scratch,
                            &mut newc,
                            bidx2(nx, ny, jx, jy, jt),
                            sc * wt * wy * wx,
                        );
                    }
                }
            }
        }
        let new_max: f32 = newc.iter().map(|&j| self.scratch[j as usize]).fold(0.0, f32::max);
        if new_max > 0.0 {
            // 等化混合: 生存 belief の最大セルを新候補の最大セルに揃えてから
            // 足す。定数比 (0.5/0.5) の混合は、濃縮した生存ピーク (誘拐検出時の
            // 誤追跡ゴースト、max ~0.3) に対し注入候補が ~1e-4 で入り、初回
            // observe の相対枝刈り (max × weight_skip_ratio) が候補を皆殺しに
            // する — flatten 時代に全平坦化で潰したゴースト持ち越しの再来
            // (津田沼 K4 で実測: 真値の注入重みが枝刈り線上)。等化なら相対
            // 順位 (淘汰の記憶) は残り、正しい追跡ピークはマッチ自身が高スコア
            // で再提案するので失うものはない。
            let old_max = self.active.iter().map(|&i| self.b[i as usize]).fold(0.0, f32::max);
            if old_max > 0.0 {
                let sf = new_max / old_max;
                for &i in &self.active {
                    self.b[i as usize] *= sf;
                }
            }
            for &j in &newc {
                let ju = j as usize;
                let add = self.scratch[ju];
                self.scratch[ju] = 0.0; // 全ゼロ不変の復元
                if add > 0.0 {
                    if self.b[ju] == 0.0 {
                        self.active.push(j);
                    }
                    self.b[ju] += add;
                }
            }
        } else {
            for &j in &newc {
                self.scratch[j as usize] = 0.0;
            }
        }
        self.normalize_active();
        if self.cfg.viterbi {
            self.vit_enter();
        }
        self.initialized = true;
        self.lost = true;
        self.probation = 0;
        self.move_since_match = 0.0;
        // 最良候補のスコアを quality に残す (診断用 — E 正規化済みの幾何平均
        // 尤度と同じスケール)。リセット直後に再リセットしないのは flatten と同じ。
        self.quality = best as f64;
        self.q_ewma = self.cfg.reset_quality;
        self.recompute_ess();
    }

    // ═══ 内部: min-plus (viterbi) ═══

    /// δ の初期化: 今の b (正規化済み) を -ln で写す (min-plus は定数シフト
    /// 不変なので正規化定数は気にしない)。b = 0 (非 free 等) は +INF。
    fn vit_enter(&mut self) {
        self.delta.fill(f32::INFINITY);
        for &i in &self.active {
            let w = self.b[i as usize];
            if w > 0.0 {
                self.delta[i as usize] = (-(w as f64).ln()) as f32;
            }
        }
    }

    /// 補正の min-plus 版: (flush 済みの) δ へ観測コスト -ln(尤度積) を加算し、
    /// b = exp(δmin − δ) を実体化する。戻り値は quality (従来と同じ
    /// 「前回 b 加重のビーム幾何平均尤度」— しきい値系をそのまま使う)。
    fn vit_observe(&mut self, beams: &[(f64, f64)], lut: Option<&[[f32; 256]]>) -> f64 {
        // 焼きなまし強度 (sum 側と同じ、borrow の都合で先に取る)。
        let apow = if self.cfg.global_match { self.anneal_pow() } else { 1 };
        let (nx, ny, nt) = (self.nx, self.ny, self.nt);
        let n = (nx as usize) * (ny as usize) * (nt as usize);
        let (ox, oy, res, t_res) = (self.ox, self.oy, self.res, self.t_res_deg);
        let z_min = self.cfg.z_min;
        // Bayes 側の weight_skip_ratio と同じ意味の枝刈り (δ は -ln 重み)。
        // ただしリセット床 ([`Belief::reset_floor_ln`]) は必ず内側に含める:
        // 床を切ってから観測を足すと、一様混合が張った仮説が尤度で評価される
        // 前に消え、再定位が二度と起きない (sum_observe が枝刈りを乗算**後**に
        // 置いているのと同じ理由)。床が張られた直後の 1 回だけ全域評価になる。
        let thr_ln =
            (-(self.cfg.weight_skip_ratio.max(1e-30)).ln()).max(self.reset_floor_ln()) as f32;
        let m_inv = 1.0 / beams.len() as f64;
        // sum 側と同じ: 追跡クランプ / 証拠不足の頭打ち定数。
        let unk_l = z_min + (1.0 - z_min) * UNKNOWN_L;
        let bt: Vec<(f64, f64, f64)> =
            beams.iter().map(|&(ba, r)| (ba.cos(), ba.sin(), r)).collect();
        let mut quality = 0.0f64;
        let delta = &mut self.delta;
        let dmin0 = delta[..n].iter().cloned().fold(f32::INFINITY, f32::min);
        for it in 0..nt {
            let th = ((it as f64 + 0.5) * t_res).to_radians();
            let (ct, st) = (th.cos(), th.sin());
            for iy in 0..ny {
                let cy = oy + (iy as f64 + 0.5) * res;
                for ix in 0..nx {
                    let i = bidx2(nx, ny, ix, iy, it);
                    let d = delta[i];
                    // NaN (全滅時の INF−INF) もこの否定形で落ちる。
                    if !(d - dmin0 <= thr_ln) {
                        delta[i] = f32::INFINITY;
                        continue;
                    }
                    if !self.free[(iy * nx + ix) as usize] {
                        delta[i] = f32::INFINITY;
                        continue;
                    }
                    let cx = ox + (ix as f64 + 0.5) * res;
                    let (mut prod, mut known) = (1.0f64, 0u32);
                    for (bi, &(cb, sb, r)) in bt.iter().enumerate() {
                        let (ca, sa) = (ct * cb - st * sb, st * cb + ct * sb);
                        let (px, py) = (cx + r * ca, cy + r * sa);
                        match lut {
                            // sum 側と同じ E 正規化 (LOST_MIN_KNOWN の doc 参照)。
                            Some(t) => {
                                if let Some(dk) = self.field.known_dist(px, py) {
                                    prod *= t[bi][dk] as f64;
                                    known += 1;
                                }
                            }
                            None => {
                                let mut lb = z_min + (1.0 - z_min) * self.field.at(px, py);
                                if lb < unk_l && self.field.unk_at(px, py) {
                                    lb = unk_l;
                                }
                                prod *= lb;
                            }
                        }
                    }
                    let (gm, cost) = if lut.is_some() {
                        let s = if known >= LOST_MIN_KNOWN {
                            prod.powf(bt.len() as f64 / known as f64)
                        } else {
                            unk_l
                        };
                        // sum 側と同じ「未知 = 中立」記数 + 焼きなまし
                        // (min-plus では cost に -ln が掛かる)。
                        let cv = if self.cfg.global_match {
                            (prod * unk_l.powf((bt.len() - known as usize) as f64 * m_inv))
                                .powf(apow as f64)
                        } else {
                            s
                        };
                        (s, cv)
                    } else {
                        (prod.powf(m_inv), prod)
                    };
                    quality += self.b[i] as f64 * gm;
                    delta[i] = d - cost.ln() as f32;
                }
            }
        }
        // b = exp(δmin − δ) の実体化 + アクティブ集合の再構成。
        let dmin = delta[..n].iter().cloned().fold(f32::INFINITY, f32::min);
        self.active.clear();
        if dmin.is_finite() {
            for (i, d) in delta[..n].iter().enumerate() {
                self.b[i] = if d.is_finite() {
                    let w = ((dmin - d) as f64).exp() as f32;
                    if w > 0.0 {
                        self.active.push(i as u32);
                    }
                    w
                } else {
                    0.0
                };
            }
        } else {
            // 全滅 (シフトで地図外へ抜けた等) — free 一様へ自己修復。
            for it in 0..nt {
                for iy in 0..ny {
                    let row = (iy * nx) as usize;
                    for ix in 0..nx {
                        let i = bidx2(nx, ny, ix, iy, it);
                        let f = self.free[row + ix as usize];
                        delta[i] = if f { 0.0 } else { f32::INFINITY };
                        self.b[i] = if f { 1.0 } else { 0.0 };
                        if f {
                            self.active.push(i as u32);
                        }
                    }
                }
            }
        }
        self.normalize_active();
        quality
    }
}

/// min-plus の決定的シフト: 各 θ 面をその方位の移動量ぶん整数シフトし、θ 面
/// 自体を回転ぶん円環シフトする (sum-product のシフトの min-plus 版 — 補間は
/// しない。半セルの誤差は直後の緩和が吸収する)。範囲外からの取り込みは +INF。
#[allow(clippy::too_many_arguments)]
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

/// 仮説集合の位置の広がり [m] — 重み付き RMS 半径 `√(σx² + σy²)`。
/// [`crate::local::ValueIteratorLocal::inflate_by_sigma`] に渡すマージン膨張量の元で、
/// 窓つき・全地図どちらの `top_cells` にもそのまま使える。
///
/// 上田ら 2023 の式(4) は `σ = ∛(σx·σy·σθ)` だが、あちらの σ はパーティクル分布の
/// 共分散なので軸方向の分散が 0 にならない。離散 belief の上位セルは 1 軸に並ぶことが
/// あり、そのとき σy = 0 で幾何平均ごと 0 になる — 位置が 2 m ばらけていてもマージンが
/// 増えない。用途は「壁からどれだけ離れるか」= 長さなので、潰れない RMS 半径を採る。
/// θ を混ぜないのも同じ理由 (向きのばらつきは離れるべき距離に効かない)。
///
/// 重みは非正規化でよい。仮説が 1 個以下なら 0。
pub fn spread_m(hyps: &[(PoseView, f64)]) -> f64 {
    if hyps.len() < 2 {
        return 0.0;
    }
    let w: f64 = hyps.iter().map(|h| h.1).sum();
    if w <= 0.0 {
        return 0.0;
    }
    let (mut mx, mut my) = (0.0, 0.0);
    for (p, wi) in hyps {
        mx += wi * p.x;
        my += wi * p.y;
    }
    let (mx, my) = (mx / w, my / w);
    let mut var = 0.0;
    for (p, wi) in hyps {
        var += wi * ((p.x - mx).powi(2) + (p.y - my).powi(2));
    }
    (var / w).max(0.0).sqrt()
}

/// 重み降順の仮説列 ([`Belief::top_cells`] の出力) を最小間隔 `min_sep_m` で
/// 間引いた「峰」。旧 AdaptiveLocalizer の `top_modes` の後継。
pub fn modes(hyps: &[(PoseView, f64)], min_sep_m: f64) -> Vec<PoseView> {
    let mut out: Vec<PoseView> = Vec::new();
    for (p, _) in hyps {
        if out.iter().all(|q| (p.x - q.x).hypot(p.y - q.y) >= min_sep_m) {
            out.push(*p);
        }
    }
    out
}

/// [`modes`] の数。
///
/// 多峰性を**セル数**で測ってはいけない: 全地図 belief のアクティブ集合は
/// 収束していても数千セルあるので `top_cells(k).len() >= 2` は常に真になり、
/// それを QMDP の発火条件にすると毎 tick QMDP = follow_controller が一度も
/// 動かない (tb3 デモで実測: ゴール手前 0.29 m で 302 s 張り付き、この関数で
/// ゲートすると 22 s で到達)。
pub fn mode_count(hyps: &[(PoseView, f64)], min_sep_m: f64) -> usize {
    modes(hyps, min_sep_m).len()
}

/// [`modes`] の重み保存版: 重み降順の仮説セル列を `min_sep_m` 分離の峰へ
/// 集約する (各セルの質量は最寄りの峰の代表セルに合算、峰は上位 `k` 個まで —
/// あぶれた新峰の質量は僅少なので捨てる)。
///
/// QMDP へ渡す仮説集合はこれを使うこと。生の top-k セルは広い地図の多峰
/// belief では数十峰に散り、Q の薄まりと veto の積み上げで `qmdp_decide` が
/// NoAction に張り付く (津田沼のロスト中で実測 94% — 集約で 0 になる)。
/// 屋内級の地図では top-k ≒ 少数峰なのでどちらでも同じ。
pub fn weighted_modes(
    hyps: &[(PoseView, f64)],
    min_sep_m: f64,
    k: usize,
) -> Vec<(PoseView, f64)> {
    let mut out: Vec<(PoseView, f64)> = Vec::new();
    for &(p, w) in hyps {
        if let Some((_, ow)) =
            out.iter_mut().find(|(q, _)| (p.x - q.x).hypot(p.y - q.y) < min_sep_m)
        {
            *ow += w;
        } else if out.len() < k {
            out.push((p, w));
        }
    }
    out
}

/// 能動的再定位の判別変位: 上位モード仮説 {pᵢ} はオドメトリ共有で「同じロボット系
/// 変位 δ で一緒に動く」ので、δ 先の地図が仮説間で最も違う δ* を選べば、そこへ
/// 走るだけで観測が仮説を判別する:
///
///   δ* = argmax_δ Σ_{i<j} ‖sig_i(δ) − sig_j(δ)‖₁
///
/// sig は δ 先周りの尤度場リング標本 (仮説の向きに合わせて回す = 擬似的な期待
/// スキャン)。全仮説の δ 先が free な δ だけ許す (どの仮説が真でも行ける行き先)。
/// 返すのは仮説ごとの行き先 1 点。スコア 0 (完全対称) と仮説 1 個以下は空 —
/// 受動復帰に任せる。
///
/// 尤度場は窓つき ([`crate::localize::AdaptiveLocalizer`]) と全地図 ([`Belief`])
/// で別の型なので、free 判定と尤度参照だけをクロージャで受ける。
pub fn reloc_targets(
    modes: &[PoseView],
    free_at: impl Fn(f64, f64) -> bool,
    lf_at: impl Fn(f64, f64) -> f64,
    scale: f64,
) -> Vec<(f64, f64)> {
    use std::f64::consts::PI;
    /// 候補変位の半径 [m]、ロボット系方位の分割数、署名リングの半径 [m]。
    /// いずれも `scale` ([`BeliefConfig::reloc_scale`]) 倍で使う: 基準値は
    /// 屋内 (TB3 級、壁まで 1〜2 m) 向けで、尤度場の台は σ 程度 (~1 m) しか
    /// ないため、クリアランス 3 m 級の屋外道路では署名リングが全モードで
    /// 0 になりスコア 0 (= 判別不能扱い) に潰れる — 屋外はスケールを上げる。
    const RADII: [f64; 2] = [1.5, 3.0];
    const HEADINGS: usize = 12;
    const SIG_R: f64 = 1.0;
    let radii = [RADII[0] * scale, RADII[1] * scale];
    let sig_r = SIG_R * scale;

    if modes.len() < 2 {
        return Vec::new();
    }
    let displaced = |p: &PoseView, dr: f64, dphi: f64| {
        let a = p.yaw_rad + dphi;
        (p.x + dr * a.cos(), p.y + dr * a.sin())
    };
    let mut best: Option<(f64, f64, f64)> = None; // (score, dr, dphi)
    for &dr in &radii {
        for k in 0..HEADINGS {
            let dphi = k as f64 * (2.0 * PI / HEADINGS as f64);
            if !modes.iter().all(|p| {
                let (x, y) = displaced(p, dr, dphi);
                free_at(x, y)
            }) {
                continue;
            }
            let sigs: Vec<[f64; 8]> = modes
                .iter()
                .map(|p| {
                    let (x, y) = displaced(p, dr, dphi);
                    let mut s = [0.0; 8];
                    for (j, sv) in s.iter_mut().enumerate() {
                        let a = p.yaw_rad + j as f64 * (2.0 * PI / 8.0);
                        *sv = lf_at(x + sig_r * a.cos(), y + sig_r * a.sin());
                    }
                    s
                })
                .collect();
            let mut score = 0.0;
            for i in 0..sigs.len() {
                for j in (i + 1)..sigs.len() {
                    for m in 0..8 {
                        score += (sigs[i][m] - sigs[j][m]).abs();
                    }
                }
            }
            if best.map_or(true, |(bs, ..)| score > bs) {
                best = Some((score, dr, dphi));
            }
        }
    }
    match best {
        Some((score, dr, dphi)) if score > 0.0 => {
            modes.iter().map(|p| displaced(p, dr, dphi)).collect()
        }
        _ => Vec::new(),
    }
}

/// 真値姿勢からの全周スキャンをレイマーチで合成する理想センサ (angle_min = 0)。
/// テストと `viola_bench` の閉ループシミュレーションが共用する。
pub fn cast_scan(g: &OccupancyGrid, truth: PoseView, n_beams: usize, max_r: f64) -> LaserScan {
    let inc = 2.0 * std::f64::consts::PI / n_beams as f64;
    let step = g.resolution / 2.0;
    let ranges = (0..n_beams)
        .map(|i| {
            let a = truth.yaw_rad + i as f64 * inc;
            let mut r = step;
            loop {
                if r >= max_r {
                    break max_r;
                }
                let ix = ((truth.x + r * a.cos() - g.origin_x) / g.resolution).floor() as i32;
                let iy = ((truth.y + r * a.sin() - g.origin_y) / g.resolution).floor() as i32;
                if ix < 0 || iy < 0 || ix >= g.width || iy >= g.height {
                    break max_r;
                }
                if g.data[(iy * g.width + ix) as usize] != 0 {
                    break r;
                }
                r += step;
            }
        })
        .collect();
    LaserScan { angle_min: 0.0, angle_increment: inc, ranges }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Quaternion;

    fn pose(x: f64, y: f64, yaw: f64) -> PoseView {
        PoseView { x, y, yaw_rad: yaw }
    }

    /// 外周を壁で囲い、対称性崩しの内部ブロックを 1 つ置いた占有格子 (@0.05 m)。
    fn walled_grid(size: i32) -> OccupancyGrid {
        let mut g = OccupancyGrid {
            width: size,
            height: size,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (size * size) as usize],
        };
        for i in 0..size {
            for (x, y) in [(i, 0), (i, size - 1), (0, i), (size - 1, i)] {
                g.data[(y * size + x) as usize] = 100;
            }
        }
        for y in 10..14 {
            for x in 40..44 {
                g.data[(y * size + x) as usize] = 100;
            }
        }
        g
    }

    /// 10m×10m、非対称な内部構造つきの占有格子 (@0.05 m)。ブロック配置が
    /// 場所ごとに違うスキャン特徴を作るので、大域再定位が一意に解ける。
    fn tenm_grid() -> OccupancyGrid {
        let size = 200;
        let mut g = OccupancyGrid {
            width: size,
            height: size,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (size * size) as usize],
        };
        for i in 0..size {
            for (x, y) in [(i, 0), (i, size - 1), (0, i), (size - 1, i)] {
                g.data[(y * size + x) as usize] = 100;
            }
        }
        for (x0, x1, y0, y1) in
            [(20, 30, 20, 30), (140, 170, 40, 48), (60, 66, 150, 180), (118, 126, 118, 126)]
        {
            for y in y0..y1 {
                for x in x0..x1 {
                    g.data[(y * size + x) as usize] = 100;
                }
            }
        }
        g
    }

    fn wrap_rad(d: f64) -> f64 {
        use std::f64::consts::PI;
        (d + PI).rem_euclid(2.0 * PI) - PI
    }

    /// ずらしたシードから合成スキャンで真値へ収束すること (correct の本体)。
    #[test]
    fn belief_tightens_onto_the_true_pose() {
        let g = walled_grid(60); // 3m×3m @0.05
        let truth = pose(1.2, 1.5, 0.5);
        let bc = BeliefConfig {
            beam_step: 1,
            init_sigma_xy_m: 0.3,
            init_sigma_theta_deg: 20.0,
            ..BeliefConfig::default()
        };
        let mut loc = Belief::new(&g, 36, &g, bc);
        assert!(loc.pose().is_none(), "シード前は None");

        // 真値から 0.14m / 11° ずらして手動シード。
        loc.seed(pose(1.3, 1.4, 0.3));
        let scan = cast_scan(&g, truth, 36, 5.0);
        for _ in 0..6 {
            loc.observe(&scan);
            loc.predict(0.0, 0.0, 0.1); // 静止でも動作ノイズの拡散は回る
        }

        let m = loc.pose().expect("収束後の平均");
        assert!(
            (m.x - truth.x).abs() < 0.1 && (m.y - truth.y).abs() < 0.1,
            "mean ({:.3}, {:.3}) が真値 ({:.3}, {:.3}) から遠い",
            m.x, m.y, truth.x, truth.y
        );
        assert!(
            wrap_rad(m.yaw_rad - truth.yaw_rad).abs() < 0.2,
            "yaw {:.3} が真値 {:.3} から遠い",
            m.yaw_rad, truth.yaw_rad
        );
        // 観測一致度は AdaptiveLocalizer 由来の「z_min 混合後のビーム**幾何**
        // 平均」(旧 GridLocalizer の生尤度の算術平均ではない — 幾何平均でないと
        // ロストと追従がしきい値で分離できない、という理由でこちらを採った)。
        // 同じ収束状態でも値域が違うので、意味のある主張は「リセット
        // (= ロスト) しきい値から十分上」。
        let reset_q = BeliefConfig::default().reset_quality;
        assert!(
            loc.quality() > 1.5 * reset_q,
            "観測一致度が低すぎる: {} (リセットしきい値 {})",
            loc.quality(),
            reset_q
        );
    }

    /// predict が指令どおり平均を進め、回すこと (動作モデル = 自分の cmd_vel)。
    /// flush 前は残 pend の解析加算、observe (flush) 後は belief 本体のシフト。
    #[test]
    fn belief_predict_advances_the_mean_along_the_heading() {
        let g = walled_grid(60);
        // ESS ゲートに掛からない程度に締めた初期 belief (observe 無しで pose を読む)。
        let bc = BeliefConfig {
            init_sigma_xy_m: 0.1,
            init_sigma_theta_deg: 5.0,
            ..BeliefConfig::default()
        };
        let mut loc = Belief::new(&g, 36, &g, bc);
        loc.seed(pose(1.5, 1.5, 0.0));

        // 前進 0.3 m/s × 1.0 s。flush 前でも pose() は解析 pend で前進を返す。
        for _ in 0..10 {
            loc.predict(0.3, 0.0, 0.1);
        }
        let m = loc.pose().expect("解析 pend の平均");
        assert!((m.x - 1.8).abs() < 0.08, "x = {:.3} (期待 1.8 付近)", m.x);
        assert!((m.y - 1.5).abs() < 0.05, "y = {:.3} (期待 1.5 のまま)", m.y);

        // observe が flush を適用 — belief 本体も前進しているはず。
        let scan = cast_scan(&g, pose(1.8, 1.5, 0.0), 36, 5.0);
        loc.observe(&scan);
        let m = loc.pose().expect("flush 後の平均");
        assert!((m.x - 1.8).abs() < 0.1, "flush 後 x = {:.3} (期待 1.8 付近)", m.x);

        // その場旋回 90 deg/s × 1.0 s (解析 pend)。
        for _ in 0..10 {
            loc.predict(0.0, 90.0, 0.1);
        }
        let m = loc.pose().expect("平均");
        assert!(
            wrap_rad(m.yaw_rad - std::f64::consts::FRAC_PI_2).abs() < 0.2,
            "yaw = {:.3} (期待 π/2 付近)",
            m.yaw_rad
        );
    }

    /// 誘拐 (瞬間移動) から一様混合リセットで復帰すること。ロスト検出
    /// (ESS ゲートで pose None) → リセット → 再定位、の全カスケード。
    #[test]
    fn belief_recovers_from_kidnap() {
        let g = tenm_grid();
        let bc = BeliefConfig { beam_step: 4, ..BeliefConfig::default() };
        let mut loc = Belief::new(&g, 36, &g, bc);

        let a = pose(2.5, 2.0, 0.4);
        loc.seed(a);
        let scan_a = cast_scan(&g, a, 180, 12.0);
        for _ in 0..5 {
            loc.observe(&scan_a);
        }
        assert!(loc.pose().is_some());

        // 誘拐: 約 8m 離れた場所へ。
        let b = pose(8.0, 8.0, 2.0);
        let scan_b = cast_scan(&g, b, 180, 12.0);
        let (mut lost, mut recovered) = (false, None);
        for i in 0..150 {
            loc.observe(&scan_b);
            match loc.pose() {
                None => lost = true,
                Some(p) => {
                    if lost
                        && (p.x - b.x).abs() < 0.4
                        && (p.y - b.y).abs() < 0.4
                        && wrap_rad(p.yaw_rad - b.yaw_rad).abs() < 0.4
                    {
                        recovered = Some(i);
                        break;
                    }
                }
            }
        }
        assert!(lost, "誘拐でロスト (pose None) を検出すること");
        assert!(
            recovered.is_some(),
            "150 スキャン以内に再定位すること (q={:.3}, ess={:.0})",
            loc.quality(),
            loc.ess()
        );
    }

    /// 未シードでも最初のスキャンから大域初期化で立ち上がること。
    #[test]
    fn belief_global_init_without_seed() {
        let g = tenm_grid();
        let bc = BeliefConfig { beam_step: 4, ..BeliefConfig::default() };
        let mut loc = Belief::new(&g, 36, &g, bc);
        assert!(loc.pose().is_none());
        let truth = pose(8.0, 8.0, 2.0);
        let scan = cast_scan(&g, truth, 180, 12.0);
        let mut ok = None;
        for i in 0..150 {
            loc.observe(&scan);
            if let Some(p) = loc.pose() {
                if (p.x - truth.x).abs() < 0.4 && (p.y - truth.y).abs() < 0.4 {
                    ok = Some(i);
                    break;
                }
            }
        }
        assert!(ok.is_some(), "未シードでも大域初期化で再定位すること (ess={:.0})", loc.ess());
    }

    /// 推定が壁・未知の中に落ちないこと (free マスク + free スナップ)。
    /// ブロックのど真ん中へシードすると、マスクが質量を周囲の free へ追い出し、
    /// リング状に残った belief の平均はブロック内 (穴) に戻る — pose() は free 上の
    /// mode へ吸着して返すはず。マスクかスナップのどちらが欠けても落ちる。
    #[test]
    fn estimate_never_lands_in_occupied_space() {
        let g = walled_grid(80);
        let free_at = |p: PoseView| {
            let ix = ((p.x - g.origin_x) / g.resolution).floor() as i32;
            let iy = ((p.y - g.origin_y) / g.resolution).floor() as i32;
            (0..g.width).contains(&ix)
                && (0..g.height).contains(&iy)
                && g.data[(iy * g.width + ix) as usize] == 0
        };
        // walled_grid の内部ブロック (x40..44, y10..14) の中心。ESS ゲートに
        // 掛からない程度に締めた初期 belief。
        let block_center = pose(42.0 * 0.05, 12.0 * 0.05, 0.0);
        let bc = BeliefConfig {
            init_sigma_xy_m: 0.1,
            init_sigma_theta_deg: 5.0,
            ..BeliefConfig::default()
        };
        let mut loc = Belief::new(&g, 36, &g, bc);
        loc.seed(block_center);
        let p = loc.pose().expect("マスク後も free 側に質量が残ること");
        assert!(free_at(p), "推定 ({:.2}, {:.2}) が free でない", p.x, p.y);
    }

    /// viterbi (min-plus): 誘拐から復帰すること — sum-product をやめて全期間
    /// 再帰 MAP にしても同じ復帰性が保たれる回帰。
    #[test]
    fn viterbi_recovers_from_kidnap() {
        let g = tenm_grid();
        let bc = BeliefConfig { beam_step: 4, viterbi: true, ..BeliefConfig::default() };
        let mut loc = Belief::new(&g, 36, &g, bc);

        let a = pose(2.5, 2.0, 0.4);
        loc.seed(a);
        let scan_a = cast_scan(&g, a, 180, 12.0);
        for _ in 0..5 {
            loc.observe(&scan_a);
        }
        assert!(loc.pose().is_some());

        let b = pose(8.0, 8.0, 2.0);
        let scan_b = cast_scan(&g, b, 180, 12.0);
        let (mut lost, mut recovered) = (false, None);
        for i in 0..150 {
            loc.observe(&scan_b);
            match loc.pose() {
                None => lost = true,
                Some(p) => {
                    if lost
                        && (p.x - b.x).abs() < 0.4
                        && (p.y - b.y).abs() < 0.4
                        && wrap_rad(p.yaw_rad - b.yaw_rad).abs() < 0.4
                    {
                        recovered = Some(i);
                        break;
                    }
                }
            }
        }
        assert!(lost, "誘拐でロスト (pose None) を検出すること");
        assert!(
            recovered.is_some(),
            "150 スキャン以内に再定位すること (q={:.3}, ess={:.0})",
            loc.quality(),
            loc.ess()
        );
    }

    /// viterbi: 未シードの大域初期化 (enter_uniform_free → δ 一様) でも
    /// 立ち上がること。ロスト中の predict は移動量の記録だけ (O(1))。
    #[test]
    fn viterbi_global_init_without_seed() {
        let g = tenm_grid();
        let bc = BeliefConfig { beam_step: 4, viterbi: true, ..BeliefConfig::default() };
        let mut loc = Belief::new(&g, 36, &g, bc);
        assert!(loc.pose().is_none());
        let truth = pose(8.0, 8.0, 2.0);
        let scan = cast_scan(&g, truth, 180, 12.0);
        let mut ok = None;
        for i in 0..150 {
            loc.observe(&scan);
            // 静止でも落ちないこと (pend 累積のみ)。
            loc.predict(0.0, 0.0, 0.1);
            if let Some(p) = loc.pose() {
                if (p.x - truth.x).abs() < 0.4 && (p.y - truth.y).abs() < 0.4 {
                    ok = Some(i);
                    break;
                }
            }
        }
        assert!(ok.is_some(), "大域初期化から再定位できない (ess={:.0})", loc.ess());
    }

    /// top_cells: シード直後の最大重み仮説がシード姿勢のセルで、重みが降順な
    /// こと (QMDP の仮説集合の契約)。
    #[test]
    fn top_cells_returns_descending_hypotheses_near_the_seed() {
        let g = walled_grid(60);
        let seed = pose(1.5, 1.5, 0.0);
        let mut loc = Belief::new(&g, 36, &g, BeliefConfig::default());
        assert!(loc.top_cells(8).is_empty(), "シード前は空");
        loc.seed(seed);
        let cells = loc.top_cells(8);
        assert!(!cells.is_empty(), "仮説が空");
        assert!(cells.len() <= 8, "k を超過");
        for w in cells.windows(2) {
            assert!(w[0].1 >= w[1].1, "重みが降順でない");
        }
        let top = cells[0].0;
        assert!(
            (top.x - seed.x).abs() < 0.1 && (top.y - seed.y).abs() < 0.1,
            "最大重み仮説 ({:.2}, {:.2}) がシードから遠い",
            top.x, top.y
        );
    }

    /// mode_count: 収束した単峰 belief は 1 峰、離れた 2 山は 2 峰と数えること
    /// (QMDP の発火条件 — セル数で測ると常に多峰になる、の回帰)。
    #[test]
    fn mode_count_separates_peaks_not_cells() {
        // 単峰でもアクティブセルは多数 — セル数で多峰性を測ると常に真になる、
        // という実地図での状況をシードだけで再現する。
        let g = walled_grid(60);
        let mut loc = Belief::new(&g, 36, &g, BeliefConfig::default());
        loc.seed(pose(1.2, 1.5, 0.5));
        let hyps = loc.top_cells(64);
        assert!(hyps.len() >= 2, "セル数では常に多峰に見える (この前提が壊れたら本テストは無意味)");
        assert_eq!(mode_count(&hyps, 0.5), 1, "広がっていても峰が 1 つなら 1 峰");

        // 1 m 離れた 2 山を手で作る。
        let two = vec![
            (pose(1.2, 1.5, 0.5), 0.6),
            (pose(1.25, 1.5, 0.5), 0.2),
            (pose(2.2, 1.5, 0.5), 0.2),
        ];
        assert_eq!(mode_count(&two, 0.5), 2, "離れた 2 山は 2 峰");
    }

    /// spread_m: 単一仮説は 0、離れた 2 山は峰間距離に比例した**長さ**を返すこと。
    /// 文献の幾何平均 ∛(σx·σy·σθ) は 1 軸に並んだ時点で 0 に潰れる — その配置で
    /// マージン膨張が効かなくなるのを防ぐための選択なので、回帰として固定する。
    #[test]
    fn spread_m_measures_length_not_geometric_mean() {
        assert_eq!(spread_m(&[(pose(1.0, 1.0, 0.0), 1.0)]), 0.0, "単一仮説は広がり 0");

        // x 軸上に 1 m 離れた等重み 2 山 → RMS 半径 0.5 m (σy = σθ = 0 の配置)。
        let two = vec![(pose(1.0, 1.0, 0.0), 0.5), (pose(2.0, 1.0, 0.0), 0.5)];
        assert!((spread_m(&two) - 0.5).abs() < 1e-9, "got {}", spread_m(&two));

        // 重みが片側に寄れば広がりは縮む。
        let skewed = vec![(pose(1.0, 1.0, 0.0), 0.99), (pose(2.0, 1.0, 0.0), 0.01)];
        assert!(spread_m(&skewed) < 0.2, "got {}", spread_m(&skewed));
    }

    /// reloc_targets: 対称な 2 仮説から「地図が仮説間で違って見える方向」への
    /// 変位を選ぶこと (能動的再定位の行き先)。北側の一方にだけ障害物クラスタが
    /// ある地図で、両仮説とも北向きの行き先が返るはず (窓つき側の同名テストの
    /// 全地図 belief 版 — 峰の取り方だけが違う)。
    #[test]
    fn reloc_targets_point_toward_disambiguating_terrain() {
        // 20m×10m @0.1、開けた空間 + 仮説 A の北にだけ障害物クラスタ。
        let (w, h) = (200, 100);
        let mut g = OccupancyGrid {
            width: w,
            height: h,
            resolution: 0.1,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (w * h) as usize],
        };
        for y in 90..96 {
            for x in 40..70 {
                g.data[(y * w + x) as usize] = 100;
            }
        }
        let mut loc = Belief::new(&g, 36, &g, BeliefConfig::default());
        assert!(loc.reloc_targets().is_empty(), "未初期化は空");

        // ロスト状態を直接組む: (5.05, 5.05) と (15.05, 5.05) の 2 仮説 (θ ビン 0)。
        loc.initialized = true;
        loc.lost = true;
        let cell = |wx: f64, wy: f64| {
            let (ix, iy) = ((wx / 0.1) as i32, (wy / 0.1) as i32);
            (iy * loc.nx + ix) as u32
        };
        for (i, wt) in [(cell(5.05, 5.05), 0.6f32), (cell(15.05, 5.05), 0.4)] {
            loc.b[i as usize] = wt;
            loc.active.push(i);
        }
        let t = loc.reloc_targets();
        assert_eq!(t.len(), 2, "仮説ごとに 1 点");
        for &(x, y) in &t {
            assert!(y > 6.0, "行き先 ({x:.1}, {y:.1}) が判別地形 (北) を向いていない");
        }
        // 同じロボット系変位 δ (両仮説とも θ ビン 0) — 世界系でも同じずれ。
        assert!(
            ((t[1].0 - t[0].0) - 10.0).abs() < 0.5,
            "2 つの行き先は同じ δ で結ばれるはず: {t:?}"
        );
        loc.lost = false;
        assert!(loc.reloc_targets().is_empty(), "非ロストは空 (受動追従に任せる)");
    }

    /// ロスト解除は「集中」(ESS < contract) だけでは降りない — 離れた
    /// エイリアス 2 峰に集中したままの belief (津田沼 K4 の飽和プラトーで実測
    /// した形) は多峰なので lost を維持し、単峰に戻ってから解除すること。
    #[test]
    fn lost_release_requires_unimodal() {
        // 素の正方形の壁だけ (内部ブロックなし) — 180° 回転対称なので
        // (0.8, 0.8, 0°) と (2.2, 2.2, 180°) は観測で判別できないエイリアス。
        let size = 60;
        let mut g = OccupancyGrid {
            width: size,
            height: size,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (size * size) as usize],
        };
        for i in 0..size {
            for (x, y) in [(i, 0), (i, size - 1), (0, i), (size - 1, i)] {
                g.data[(y * size + x) as usize] = 100;
            }
        }
        let bc = BeliefConfig { beam_step: 4, init_sigma_xy_m: 0.1, ..BeliefConfig::default() };
        let mut loc = Belief::new(&g, 36, &g, bc);
        let truth = pose(0.8, 0.8, 0.0);
        loc.seed(truth);
        let scan = cast_scan(&g, truth, 36, 8.0);
        for _ in 0..4 {
            loc.observe(&scan); // q_ewma を安定させる
        }
        assert!(loc.pose().is_some());

        // 対称エイリアスに同量の質量を注入 — 集中 (ESS 小) だが 2 峰。
        // ラッチを立てて解除条件だけを試す。
        loc.lost = true;
        let peak = loc.active.iter().map(|&i| loc.b[i as usize]).fold(0.0f32, f32::max);
        let it = (180.0 / (360.0 / 36.0)) as i32; // θ ビン 18
        let (ix, iy) = ((2.2 / loc.res) as i32, (2.2 / loc.res) as i32);
        let alias = ((it * loc.ny + iy) * loc.nx + ix) as u32;
        loc.b[alias as usize] = peak;
        loc.active.push(alias);
        loc.observe(&scan);
        assert!(
            loc.ess() < loc.cfg.contract_ess,
            "前提: 2 峰でも集中はしている (ess={:.1})",
            loc.ess()
        );
        assert!(loc.pose().is_none(), "多峰のままの解除は誤姿勢を返す — 降りてはいけない");

        // エイリアスを消せば単峰 — 通常どおり解除する。
        loc.b[alias as usize] = 0.0;
        loc.observe(&scan);
        loc.observe(&scan);
        assert!(loc.pose().is_some(), "単峰 + 集中 + 観測一致で解除するはず");
    }

    /// 解除の probation: 仮解除中に予測整合が割れたら lost へ戻して再拡散し、
    /// 整合が probation_obs 回続けば確定する (誤エイリアスへの誤解除は静止では
    /// 判別できないが、走ると予測が破綻する — 津田沼で実測)。
    #[test]
    fn probation_reverts_on_mismatch_and_confirms_on_consistency() {
        let size = 60;
        let mut g = OccupancyGrid {
            width: size,
            height: size,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (size * size) as usize],
        };
        for i in 0..size {
            for (x, y) in [(i, 0), (i, size - 1), (0, i), (size - 1, i)] {
                g.data[(y * size + x) as usize] = 100;
            }
        }
        let bc = BeliefConfig {
            beam_step: 4,
            init_sigma_xy_m: 0.1,
            probation_obs: 2,
            probation_min_q: 0.3,
            // 0 = 毎観測カウント (テストは predict なしで回すため)。
            lost_update_min_d_m: 0.0,
            ..BeliefConfig::default()
        };
        let mut loc = Belief::new(&g, 36, &g, bc);
        let truth = pose(0.8, 0.8, 0.0);
        loc.seed(truth);
        let scan = cast_scan(&g, truth, 36, 8.0);
        for _ in 0..4 {
            loc.observe(&scan);
        }
        // ラッチを立てる — belief は単峰・集中のままなので次の観測で解除条件が
        // 立つが、probation_obs > 0 なので仮解除に入る。
        loc.lost = true;
        loc.observe(&scan);
        assert!(loc.pose().is_some(), "仮解除でも pose は返る");
        assert!(loc.in_probation(), "解除直後は probation 中のはず");

        // 検証 1 回目: 整合 — まだ確定しない。
        loc.observe(&scan);
        assert!(loc.in_probation(), "probation_obs=2: 1 回の整合では確定しない");

        // 2 回目に予測と割れる観測 (全ビーム 0.4 m — 壁のない空中) — lost へ
        // 戻して全域一様へ再拡散すること。
        let mut bad = scan.clone();
        for r in &mut bad.ranges {
            *r = 0.4;
        }
        loc.observe(&bad);
        assert!(loc.pose().is_none(), "予測整合が割れたら lost へ戻ること");
        assert!(!loc.in_probation());
        assert!(
            loc.ess() > loc.cfg.lost_ess,
            "失敗時は全域一様へ再拡散すること (ess={:.0})",
            loc.ess()
        );

        // 確定パス: 立ち上げ直して同じ仮解除から整合 2 回 — 確定して
        // probation が消えること。
        loc.seed(truth);
        for _ in 0..4 {
            loc.observe(&scan);
        }
        loc.lost = true;
        loc.observe(&scan); // 仮解除
        loc.observe(&scan); // 整合 1
        loc.observe(&scan); // 整合 2 — 確定
        assert!(loc.pose().is_some());
        assert!(!loc.in_probation(), "整合が続けば確定するはず");
    }

    /// ロスト中の距離比例 σ: θ ビン中心の角度誤差 (±3° @60 ビン) は長ビームの
    /// 端点を r·Δθ だけ振る — 18 m 先の孤立柱を 3° ずれて評価すると端点は
    /// ~1 m 外れ、固定 σ0 = 0.2 m では尤度 0 (床 z_min) に潰れるが、
    /// lost_sigma_per_m で σ_eff = 0.2 + 0.05·18 = 1.1 m に膨れば回復する。
    #[test]
    fn lost_sigma_per_m_forgives_theta_bin_offset_at_range() {
        // 100×100 @0.5 m の全 free 地図 + 孤立柱 1 セル。スキャンは手書き 1 本
        // (レイキャスト無し = 幾何が式のまま)。
        let size = 100;
        let mut g = OccupancyGrid {
            width: size,
            height: size,
            resolution: 0.5,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (size * size) as usize],
        };
        // 真の姿勢 (25, 25, 6°) の正面 18 m に柱。仮説は θ ビン中心 (3°) で
        // しか評価できない — 端点は柱から ~1 m 外れる。
        let (tx, ty) = (25.0 + 18.0 * 6f64.to_radians().cos(), 25.0 + 18.0 * 6f64.to_radians().sin());
        let (px, py) = ((tx / 0.5) as i32, (ty / 0.5) as i32);
        g.data[(py * size + px) as usize] = 100;
        let scan = crate::msg::LaserScan {
            angle_min: 0.0,
            angle_increment: 0.1,
            ranges: vec![18.0],
            ..Default::default()
        };
        let hyp = pose(25.0, 25.0, 3f64.to_radians()); // nt=60 の it=0 ビン中心
        let q_at = |per_m: f64| {
            let bc = BeliefConfig {
                beam_step: 1,
                max_range_m: 60.0,
                lost_sigma_per_m: per_m,
                ..BeliefConfig::default()
            };
            let mut loc = Belief::new(&g, 60, &g, bc);
            loc.seed(hyp);
            loc.lost = true; // LUT はロスト中のみ
            loc.observe(&scan);
            loc.quality()
        };
        let (q0, q1) = (q_at(0.0), q_at(0.05));
        assert!(q0 < 0.15, "固定 σ では床に潰れるはず (q0={q0:.3})");
        // seed は隣接 θ ビン (±6°) にも質量を撒き、そこは σ_eff でも半端に
        // 罰されるので中心ビン単体の ~0.68 までは戻らない — 分離だけを見る。
        assert!(q1 > 0.3 && q1 > 4.0 * q0, "距離比例 σ で回復するはず (q0={q0:.3} q1={q1:.3})");
    }

    /// ロスト中のスキャン内テンパリング: 生のビーム積は「全ビーム床 (z_min)」
    /// の仮説を 1 観測で 0.05^M ≈ 1e-8 に落とし相対枝刈りが即殺するが、
    /// 幾何平均 (LUT 焼き込み) では 1 観測の下限が z_min = 0.05 なので
    /// 枝刈り (max × 1e-4) を生き延びる — リセット直後の真位置仮説の寿命。
    #[test]
    fn lost_tempering_keeps_moderate_hypotheses_alive() {
        let size = 100;
        let mut g = OccupancyGrid {
            width: size,
            height: size,
            resolution: 0.5,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (size * size) as usize],
        };
        // 仮説 A (25, 25, 3°) の間引き後ビーム (step 4 → 角 0, 0.4, .., 2.0 rad)
        // の端点にちょうど柱を置く — A はほぼ完全一致。
        let ya = 3f64.to_radians();
        for k in 0..6 {
            let a = ya + 0.4 * k as f64;
            let (px, py) = (25.0 + 18.0 * a.cos(), 25.0 + 18.0 * a.sin());
            let (ix, iy) = ((px / 0.5) as i32, (py / 0.5) as i32);
            if ix >= 0 && iy >= 0 && ix < size && iy < size {
                g.data[(iy * size + ix) as usize] = 100;
            }
        }
        let scan = crate::msg::LaserScan {
            angle_min: 0.0,
            angle_increment: 0.1,
            ranges: vec![18.0; 24], // ×4 間引きで 6 本残る
            ..Default::default()
        };
        let hyp_a = pose(25.0, 25.0, ya);
        let hyp_b = pose(25.0, 21.0, ya); // 4 m 横 — 全ビーム床
        let alive_b = |per_m: f64| {
            let bc = BeliefConfig {
                beam_step: 1,
                max_range_m: 60.0,
                lost_sigma_per_m: per_m,
                ..BeliefConfig::default()
            };
            let mut loc = Belief::new(&g, 60, &g, bc);
            loc.seed(hyp_a);
            // B を同量のピークとして注入 (リセット直後の対等な競合を模す)。
            let peak = loc.active.iter().map(|&i| loc.b[i as usize]).fold(0.0f32, f32::max);
            let it = (ya.to_degrees() / 6.0) as i32;
            let (ix, iy) = ((hyp_b.x / 0.5) as i32, (hyp_b.y / 0.5) as i32);
            let bi = ((it * loc.ny + iy) * loc.nx + ix) as u32;
            loc.b[bi as usize] = peak;
            loc.active.push(bi);
            loc.lost = true;
            loc.observe(&scan);
            loc.probe_weight(hyp_b).0 > 0.0
        };
        assert!(!alive_b(0.0), "生のビーム積では B は 1 観測で枝刈りされるはず");
        assert!(alive_b(0.05), "テンパリングで B は枝刈りを生き延びるはず");
    }

    /// unknown (-1) は尤度場の障害物に数えない: 未知領域の深部に落ちた端点は
    /// 中立 [`UNKNOWN_L`] (満点でも床でもない)。障害物扱い (旧) は未知に面した
    /// 仮説がどんなスキャンにも満点一致するブラックホール、床 (z_min) は世界の
    /// レイキャストが unknown 境界で終端するぶん真値が出血して base 追跡まで
    /// 壊す — どちらも津田沼で実測した。
    #[test]
    fn unknown_is_neutral_in_likelihood_field() {
        let size = 60;
        let mut g = OccupancyGrid {
            width: size,
            height: size,
            resolution: 0.5,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (size * size) as usize],
        };
        // 右半分 (x > 15 m) は unknown、free 側に実壁 1 セル (端点 A の位置)。
        for iy in 0..size {
            for ix in 30..size {
                g.data[(iy * size + ix) as usize] = -1;
            }
        }
        // 仮説セル中心 (5.25, 15.25)・θ ビン中心 3° から r=5 の端点が落ちるセル。
        g.data[(31 * size + 20) as usize] = 100;
        let hyp = pose(5.0, 15.0, 3f64.to_radians());
        let q_for = |r: f64| {
            let scan = crate::msg::LaserScan {
                angle_min: 0.0,
                angle_increment: 0.1,
                ranges: vec![r],
                ..Default::default()
            };
            let bc =
                BeliefConfig { beam_step: 1, max_range_m: 60.0, ..BeliefConfig::default() };
            let mut loc = Belief::new(&g, 60, &g, bc);
            loc.seed(hyp);
            loc.observe(&scan);
            loc.quality()
        };
        let q_unk = q_for(22.0); // 端点 = 未知領域の深部 (実壁から ~17 m)
        // 旧実装 (unknown = 障害物) では ≈ 1.0 (seed 近傍の全仮説の端点が未知に
        // 落ち、全て d=0 = 満点)。床実装では ≈ z_min。中立はその間に立つ。
        assert!(
            (0.3..0.8).contains(&q_unk),
            "未知深部の端点は中立域のはず (q={q_unk:.3})"
        );
    }

    /// E 正規化 (LOST_MIN_KNOWN): 未知に落ちたビームは幾何平均の分母に
    /// 入らない — フロンティア際の仮説 (既知 4 + 未知 2、既知は全一致) は
    /// 全ビーム既知の内部仮説と同格 (パリティ)、全ビーム未知の深部フリンジは
    /// 中立で頭打ち (満点に戻らない)。
    #[test]
    fn lost_e_normalization_no_frontier_handicap() {
        let size = 100;
        let mut g = OccupancyGrid {
            width: size,
            height: size,
            resolution: 0.5,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (size * size) as usize],
        };
        // 右半分 (x > 25 m) は unknown、深部に free の島 (3×3 @ (35,15))。
        for iy in 0..size {
            for ix in 50..size {
                g.data[(iy * size + ix) as usize] = -1;
            }
        }
        for iy in 29..=31 {
            for ix in 69..=71 {
                g.data[(iy * size + ix) as usize] = 0;
            }
        }
        // 各シードの間引き後ビーム (角 3°+0.4k rad, r=10) の**既知側**端点に柱。
        let ya = 3f64.to_radians();
        let mut pillar = |sx: f64, sy: f64, ks: &[usize]| {
            for &k in ks {
                let a = ya + 0.4 * k as f64;
                let (ex, ey) = (sx + 0.25 + 10.0 * a.cos(), sy + 0.25 + 10.0 * a.sin());
                let (ix, iy) = ((ex / 0.5) as i32, (ey / 0.5) as i32);
                g.data[(iy * size + ix) as usize] = 100;
            }
        };
        pillar(10.0, 15.0, &[0, 1, 2, 3, 4, 5]); // C: 内部、全 6 ビーム既知一致
        pillar(17.0, 15.0, &[2, 3, 4, 5]); // A: 際、既知 4 一致 + 未知 2
        let scan = crate::msg::LaserScan {
            angle_min: 0.0,
            angle_increment: 0.1,
            ranges: vec![10.0; 24], // ×4 間引きで 6 本
            ..Default::default()
        };
        let q_at = |sx: f64, sy: f64| {
            let bc = BeliefConfig {
                beam_step: 1,
                max_range_m: 60.0,
                lost_sigma_per_m: 0.05,
                ..BeliefConfig::default()
            };
            let mut loc = Belief::new(&g, 60, &g, bc);
            loc.seed(pose(sx, sy, ya));
            loc.lost = true;
            loc.observe(&scan);
            loc.quality()
        };
        let q_c = q_at(10.0, 15.0);
        let q_a = q_at(17.0, 15.0);
        let q_d = q_at(35.0, 15.0); // 深部フリンジ: 全ビーム未知
        // 絶対値は seed の広がり (±セル・±θ ビンの隣接仮説が σ_eff でも半端に
        // 罰される) で希釈されるので、主張はパリティ (際 ≥ 内部 × 0.85) のみ。
        assert!(
            q_a > 0.85 * q_c && q_a > 0.3,
            "際の仮説は内部と同格のはず (a={q_a:.3} c={q_c:.3})"
        );
        assert!(
            (0.35..0.7).contains(&q_d),
            "深部フリンジは中立で頭打ちのはず (d={q_d:.3})"
        );
    }

    /// global_match: flatten の代わりに全域スキャンマッチで再シードすること。
    /// 疎な候補集合 (top-K 塊 ≪ free×θ — flatten の置き換えコスト契約) に
    /// 真値近傍の仮説が入り、以後は通常の observe だけで正しく再定位する。
    #[test]
    fn global_match_reseeds_sparse_and_relocalizes() {
        let g = tenm_grid();
        let bc = BeliefConfig {
            beam_step: 4,
            global_match: true,
            lost_sigma_per_m: 0.02,
            ..BeliefConfig::default()
        };
        let mut loc = Belief::new(&g, 36, &g, bc);
        let truth = pose(8.0, 8.0, 2.0);
        let scan = cast_scan(&g, truth, 180, 12.0);
        // 未シード → enter_lost_empty → 全滅復旧経路で match_reseed。
        loc.observe(&scan);
        // コスト契約: active は枝数 × 塊 (≤27 セル) で頭打ち — flatten の
        // 全 free×θ に戻らない。小地図では全枝が生存して flatten に近づくが、
        // キャンパス級 (枝 ~120 万) では 1〜2% に落ちるのがこの上限の意味。
        let flat = loc.free_cells() * 36;
        assert!(
            !loc.active.is_empty() && loc.active.len() <= MATCH_BRANCHES * 27,
            "再シードが候補上限を超えた: active={} (flatten なら {flat})",
            loc.active.len(),
        );
        let (tw, mw, _) = loc.probe_weight(truth);
        assert!(tw > 0.0, "真値近傍に候補が張られていない (max={mw:.2e})");
        // 以後は通常の observe (ロストモデル) が候補を選別して解除する。
        let mut ok = false;
        for _ in 0..150 {
            loc.predict(0.0, 0.0, 0.1);
            loc.observe(&scan);
            if let Some(p) = loc.pose() {
                if (p.x - truth.x).hypot(p.y - truth.y) < 0.4 {
                    ok = true;
                    break;
                }
            }
        }
        assert!(ok, "マッチ再シードから再定位しない (ess={:.0})", loc.ess());
    }

    /// ローカル再マッチの窓: `match_candidates(.., Some(center))` の候補は
    /// 全て center の近傍に収まり、窓なしはそれより遠くへも張る。追跡破綻の
    /// 一次対応が遠方エイリアスを注入しないことの契約。
    #[test]
    fn local_rematch_window_bounds_candidates() {
        let g = tenm_grid();
        let bc = BeliefConfig {
            beam_step: 4,
            global_match: true,
            lost_sigma_per_m: 0.02,
            ..BeliefConfig::default()
        };
        let mut loc = Belief::new(&g, 36, &g, bc);
        let truth = pose(8.0, 8.0, 2.0);
        loc.seed(truth);
        let scan = cast_scan(&g, truth, 180, 12.0);
        let beams: Vec<(f64, f64)> = scan
            .ranges
            .iter()
            .enumerate()
            .step_by(4)
            .filter_map(|(i, &r)| {
                (r.is_finite() && r > 0.0)
                    .then(|| (scan.angle_min + scan.angle_increment * i as f64, r))
            })
            .collect();
        let win = loc.match_candidates(&beams, Some((truth.x, truth.y)));
        assert!(!win.is_empty(), "真値近傍に候補がない");
        // ブロック代表で絞るので余裕はブロック対角ぶん。
        let slack = LOCAL_REMATCH_R_M + 2.0 * MATCH_STRIDE_CELLS as f64 * loc.res;
        let far = win
            .iter()
            .map(|&(_, iu)| {
                let (ix, iy, _) = loc.decode(iu);
                let (cx, cy) = loc.cell_center(ix, iy);
                (cx - truth.x).hypot(cy - truth.y)
            })
            .fold(0.0f64, f64::max);
        assert!(far <= slack, "窓外の候補が漏れた: {far:.1} m > {slack:.1} m");
        let all = loc.match_candidates(&beams, None);
        // 正しい場所での引き直しは全域最良に見劣りしない (比較採用が成立する)。
        let best = win.iter().map(|c| c.0).fold(0.0f32, f32::max);
        let best_all = all.iter().map(|c| c.0).fold(0.0f32, f32::max);
        assert!(
            best > 0.0 && best >= LOCAL_REMATCH_ACCEPT * best_all,
            "真値近傍の最良 {best:.2} が全域最良 {best_all:.2} に見劣りする"
        );
        let far_all = all
            .iter()
            .map(|&(_, iu)| {
                let (ix, iy, _) = loc.decode(iu);
                let (cx, cy) = loc.cell_center(ix, iy);
                (cx - truth.x).hypot(cy - truth.y)
            })
            .fold(0.0f64, f64::max);
        assert!(far_all > slack, "窓なしが窓ありと同じ範囲しか張っていない");
    }

    /// ロスト中の相関観測ゲート: 静止のままの再スキャンは積分されない
    /// (pend が flush されない)。しきい値を超えて動けば積分される。
    #[test]
    fn lost_gate_skips_stationary_scans() {
        let g = tenm_grid();
        let bc = BeliefConfig {
            beam_step: 4,
            lost_update_min_d_m: 0.2,
            lost_update_min_a_deg: 30.0,
            ..BeliefConfig::default()
        };
        let mut loc = Belief::new(&g, 36, &g, bc);
        let truth = pose(2.5, 2.0, 0.4);
        loc.seed(truth);
        let scan = cast_scan(&g, truth, 90, 12.0);
        loc.observe(&scan); // 非ロスト — ゲートは効かない (従来どおり積分)
        assert_eq!(loc.pend_ticks, 0, "非ロストの観測は flush される");

        loc.lost = true;
        loc.predict(0.1, 0.0, 0.1); // 0.01 m — しきい値未満
        loc.observe(&scan);
        assert_eq!(loc.pend_ticks, 1, "ロスト中の静止スキャンは読み捨て (flush されない)");
        loc.predict(0.3, 0.0, 1.0); // +0.3 m — しきい値超え
        loc.observe(&scan);
        assert_eq!(loc.pend_ticks, 0, "動いたら積分される");
    }

    /// ESS が pose のゲートと b_hat の広がり報告を担うこと:
    /// free 一様 ⇒ ESS 大・pose None・b_hat 上端、シード + 補正 ⇒ ESS 小・
    /// pose 有り・b_hat 下端。
    #[test]
    fn ess_gates_pose_and_reports_spread() {
        let g = walled_grid(60);
        let bc = BeliefConfig {
            beam_step: 1,
            init_sigma_xy_m: 0.1,
            init_sigma_theta_deg: 5.0,
            ..BeliefConfig::default()
        };
        let mut loc = Belief::new(&g, 36, &g, bc);

        loc.enter_uniform_free();
        assert!(loc.ess() > 500.0, "一様 belief の ESS が小さすぎる: {:.0}", loc.ess());
        assert!(loc.pose().is_none(), "ロスト (ESS 超過) では pose は None");
        assert_eq!(loc.b_hat(4), 3, "ロスト ⇔ 上端ビン");

        let truth = pose(1.2, 1.5, 0.5);
        loc.seed(pose(1.3, 1.4, 0.3));
        let scan = cast_scan(&g, truth, 36, 5.0);
        for _ in 0..6 {
            loc.observe(&scan);
        }
        assert!(loc.ess() < 60.0, "収束後の ESS が大きすぎる: {:.1}", loc.ess());
        assert!(loc.pose().is_some(), "集中した belief は pose を返す");
        assert_eq!(loc.b_hat(4), 0, "集中 ⇔ 下端ビン (ess={:.1})", loc.ess());
    }

    /// 可視化グリッド: シード前は None、シード後はピークがシード位置に立ち、
    /// 質量ゼロのセルは 0 (RViz で透過)。
    #[test]
    fn grid_draws_the_marginal_with_its_peak_at_the_seed() {
        let g = walled_grid(60); // 3m×3m @0.05
        let mut loc = Belief::new(&g, 36, &g, BeliefConfig::default());
        assert!(loc.grid().is_none(), "シード前はグリッド無し");

        let seed = pose(1.3, 1.4, 0.3);
        loc.seed(seed);
        let vg = loc.grid().expect("シード後のグリッド");
        assert_eq!((vg.width, vg.height), (g.width, g.height), "VI と同じ格子");
        let (i, &v) = vg.data.iter().enumerate().max_by_key(|(_, &v)| v).unwrap();
        let (px, py) = (
            vg.origin_x + (i as i32 % vg.width) as f64 * vg.resolution,
            vg.origin_y + (i as i32 / vg.width) as f64 * vg.resolution,
        );
        assert!(v <= 98, "スケール上限を超えた: {v}");
        assert!(
            (px - seed.x).abs() < 0.1 && (py - seed.y).abs() < 0.1,
            "ピーク ({px:.2}, {py:.2}) がシードから遠い"
        );
        assert!(vg.data.iter().any(|&d| d == 0), "質量ゼロのセルが透過 (0) になっていない");
    }

    /// 地図の幽霊壁を、ビームが貫通した分だけ belief 側でも開く。VI 側だけ
    /// 開けても予測が非 free の質量を捨てるので推定姿勢が壁を越えられない。
    #[test]
    fn clear_free_from_scan_opens_the_ghost_wall_for_the_belief() {
        let mut g = walled_grid(60); // 3m×3m @0.05
        // y = 1.5 m と 2.25 m に幽霊壁を 1 行ずつ (実世界には無い)。ビームは
        // 手前の 1 本だけを貫通し、奥の 1 本には届かない。
        for x in 1..59 {
            g.data[(30 * 60 + x) as usize] = 100;
            g.data[(45 * 60 + x) as usize] = 100;
        }
        let bc = BeliefConfig {
            beam_step: 1,
            init_sigma_xy_m: 0.01,
            init_sigma_theta_deg: 0.5,
            ..BeliefConfig::default()
        };
        let mut loc = Belief::new(&g, 36, &g, bc);
        loc.seed(pose(1.525, 1.025, std::f64::consts::FRAC_PI_2)); // +y 向き、壁の 0.5 m 手前 (セル中心)

        let free_at = |l: &Belief, x: f64, y: f64| {
            let ix = ((x - l.ox) / l.res).floor() as i32;
            let iy = ((y - l.oy) / l.res).floor() as i32;
            l.free[(iy * l.nx + ix) as usize]
        };
        assert!(!free_at(&loc, 1.525, 1.525), "地図では壁");

        // 実世界のスキャン: +y へ 1.2 m 先まで素通り (幽霊壁を貫通)。
        let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![1.2] };
        loc.clear_free_from_scan(&scan);

        assert!(free_at(&loc, 1.525, 1.525), "ビームが貫通したセルが開くこと");
        assert!(loc.field.free_at(1.525, 1.525), "尤度場側の free マスクも開くこと");
        assert!(!free_at(&loc, 1.525, 2.275), "ヒット点 (2.225 m) より先の壁は開けないこと");
        assert!(loc.n_free > 0, "free 数が更新されていること");
    }
}
