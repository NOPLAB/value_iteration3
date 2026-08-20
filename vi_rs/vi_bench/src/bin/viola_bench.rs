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
//! 速めただけ)。終端の壁は尤度場の場所弁別力。
//!
//! 解除の probation (`--probation 100 --probation-min-q 0.3` →
//! `BeliefConfig::probation_obs`) も実装・計測済み (2026-08-20): 誤解除は
//! 想定どおり捕捉できる — 瞬時 quality の減衰 (誤解除でも走行 ~50 m は
//! 0.9 前後を保ってから落ちる) と、probation 中の方策飢餓 (誤姿勢が非 free
//! に落ちる) の 2 経路で lost へ戻し、全域一様へ再拡散して探索をやり直す。
//! これで終端状態は「誤走行のまま NO_ACTION で死ぬ」から「リトライ継続
//! (TIMEOUT) か安全停止 (LOST)」に変わった (K1 で判別走行 106.7 m を継続)。
//! 当初 relock 0/4 のままで、真値プローブ ([truth] トレース) による解剖で
//! 原因の連鎖を 1 つずつ特定・修正した (すべて `--lost-sigma-per-m 0.05` で
//! 有効になるロスト観測モデル + vi_lib 側の修正):
//!
//! 1. **距離比例 σ** (`lost_sigma_per_m`): セル中心・θ ビン中心の離散化が
//!    60 m ビーム端点を ±3 m 振る → LUT で σ_eff = σ0 + r·k。
//! 2. **スキャン内テンパリング**: 生のビーム積は相対枝刈りと組み合うと
//!    「ビームあたり 25% 劣るだけ」の真値を 2 観測 (0.4 m) で永久に殺す
//!    (winner's curse) → 幾何平均化で 1 観測のレンジを [z_min, 1] に圧縮。
//! 3. **unknown = 障害物の廃止**: 地図端の未知領域に囲まれた free の島が
//!    全ビーム尤度 1.0 の「ブラックホール」になり全誤解除がそこへ吸われて
//!    いた → 三値地図 (`build_occupancy_tri`) + E 正規化 (未知・地図外の
//!    端点は幾何平均の分母から除外、既知 4 ビーム未満は中立 0.5 で頭打ち)。
//!    床 (z_min) 案は世界レイキャストが unknown 境界で終端するぶん真値が
//!    出血して base 追跡まで壊した — 中立でも定数で数える限り際の真値が
//!    地図内エイリアスに恒常的に負ける。除外だけが対称。
//! 4. **回頭コミット** (`ctrl::lost_creep` の last_rot_deg): 左右の開口で
//!    最小回頭の勝者が毎 tick 反転する袋小路ディザ (±5° — 相関ゲート 30° を
//!    超えず belief まで凍結) → 前進できるまで同方向を維持。
//! 5. **リセットループの ESS ゲート**: 一様混合直後の加重平均 quality は
//!    「ゴミの平均」で恒常的に reset_quality 未満 → 毎観測リセットで濃縮が
//!    始まらない → ロスト中かつ ESS > lost_ess の間は再リセットしない。
//! 6. **ロスト中の再リセットは mix でなく全平坦化**: mix は誤ピーク
//!    (ゴースト) に質量を残したまま床を 1e-8 で張るので、床上の真値は
//!    相対枝刈りの 3 桁下から始まり確立できない → `enter_uniform_free`。
//! 7. **解除条件を単峰性のみに**: 軟化した観測モデルでは勝者シェアが ~1% で
//!    頭打ちし ESS は二度と contract_ess を割らない (真値が 127 s argmax の
//!    まま解除されずタイムアウト) → mode_count ≤ 1 だけで解除し、誤単峰は
//!    probation に検証させる分業。
//!
//! 結果 (2026-08-20、--probation 100 --probation-min-q 0.3 --lost-sigma-per-m
//! 0.05 追加): **relock 4/4** (従来 0/36) — K4 交差点 96.9 s → ゴール到達、
//! K2 広場 56.3 s (後続の再ロストからの復帰走行中に衝突 — 安全性は次の課題)、
//! K3 576 s / K1 廊下 806.8 s (並進対称を破るのに複数サイクル要、ゴールは
//! tick 予算切れ)。ロスト中 observe は平坦化後の全域走査で最大 ~20 s。
//!
//! 全域相関スキャンマッチ (`--global-match` → `BeliefConfig::global_match`、
//! 2026-08-20 後半): flatten の全域走査コスト (ロスト中 observe 最大 ~20 s =
//! 実機 40 ms 予算の 500 倍) を、粗ブロック σ 繰り込み × 全 θ → 枝勝者シード
//! の 2 段マッチ (1 回 ~1〜3 s) + 疎な候補集合の observe (~5〜40 ms) に
//! 置き換える新アーキテクチャ。8 ラウンドの計測反復で潰した設計問題:
//! (a) min-pool 上界は密集域で同点プラトー化し真値が候補に入らない →
//! ブロック中心の実距離 + ブロック半径の σ 繰り込み; (b) 全体 top-K セル
//! 選抜は高得点プラトーが席を独占する winner's curse の再来 → 枝 (ブロック
//! × θ) ごとの勝者 1 セル; (c) 毎不一致の再マッチが淘汰の記憶を破壊し勝者が
//! テレポート → 3 m レート制限 + 等化混合 (定数比 0.5 混合は検出時ゴースト
//! max ~0.3 が注入候補 ~1e-4 を相対枝刈りで皆殺しにする); (d) top-64 単峰
//! 判定が塊構造で誤爆 (真値 10% 生存中に解除) と凍結 (シェア・ESS 系は
//! テンパリング床で頭打ち) の両側に外れる → 相対峰 (max の 5%) + ピーク保持
//! 10 観測; (e) 検出時 mix_uniform の 30M セル flood (~14 s) → informed
//! 再マッチ; (f) 復帰中の方策飢餓 NO_ACTION 死 → 解除を棄却して再マッチ。
//! さらに 2 段の追い込みで確定 (2026-08-20 最終): (g) 証拠強度は固定でなく
//! **焼きなまし** (gm^pow、pow = 1 → 8 を再マッチからの走行距離で) — 軟らかい
//! 固定は勝者フリップで保持が満ちず真値で凍結、硬い固定は注入直後の真値を
//! 離散化ノイズ増幅で即殺し、σ√pow 補償はガウス部分が打ち消して中立化する
//! (全部実測); (h) **重みは「未知ビーム = 中立」で記数し直す** — E 正規化 gm
//! は少数ビーム候補ほど高分散で、良く合う 5/72 ビームの縁セル (gm ~0.9) が
//! 全証拠の真値 (~0.7) を恒常的に上回る (北東フリンジの高品質エイリアス、
//! probation q=0.63 の正体)。quality は E 正規化 gm のまま (検出しきい値系は
//! 不変)。判別点は到着ごとに引き直す (凍結保持は残りのエピソードを盲目の
//! 乱歩にする)。安全ゲートは前進成分のみ抑止 (全停止は正直な近接旋回を
//! 飢餓 → 再ロストへ落とす)。
//! 結果 (--probation-min-q 0.4 --reloc-timeout-s 900 --max-ticks 18000):
//! **relock 4/4 — K2 17.6 / K3 23.1 / K1 24.2 / K4 41.7 s** (flatten: 56.3 /
//! 576 / 806.8 / 96.9 s — 3〜30 倍高速)、K1・K3 はゴール到達 (K1 err rms
//! 0.92 m / clr min 0.90 m)。K2 は tick 上限時点で真値へ再収束中、K4 は後半の
//! 再ロスト エピソードが遠方エイリアスに絡まり reloc タイムアウト凍結で
//! LOST。対照実験 (誘拐なしで同じ地点から走行) による切り分け: K2 ルートは
//! belief 単独で完走 (419 s、err rms 0.08 m — K2 の churn は誘拐後遺症)、
//! **K4 ルートは誘拐なし・フルスタックでも TIMEOUT** (低特徴区間の一時破綻 →
//! 復帰サイクルの繰り返し) — 誘拐とは独立な**ルート追跡の頑健性**の問題。
//! もう 1 つの発見: creep なし構成は err rms 0.18 m の完璧な追跡中の一時破綻
//! から「再シード → 停止 → 相関ゲートが静止スキャンを全読み捨て → belief
//! 永久凍結」のデッドロックに落ちる — 静止反復観測の読み捨て自体は正しい
//! 設計 (誤単峰収束を実測済み) なので、**lost 中は動く (creep) ことが系の
//! 必須要件**。実ノードの `localizer: belief` に creep / active_reloc を
//! 配線するまで実機でも起きる。

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
    /// 解除の probation (`BeliefConfig::probation_obs` — belief/viterbi のみ):
    /// 仮解除後、--lost-min-d の並進を伴う観測 n 回すべてで瞬時 quality ≥
    /// --probation-min-q を確認してから確定。割れたら lost へ戻して再拡散。
    /// 0 = 従来 (即確定)。
    #[arg(long, default_value_t = 0)]
    probation: u32,
    #[arg(long, default_value_t = 0.4)]
    probation_min_q: f64,
    /// ロスト中の距離比例 σ (`BeliefConfig::lost_sigma_per_m` — belief/viterbi
    /// のみ): σ_eff = σ0 + r·この値。セル中心・θ ビン中心の離散化が長ビーム
    /// 端点を振るぶんの繰り込み。θ ビン半幅 3° ≒ 0.05 が目安。0 = 従来。
    #[arg(long, default_value_t = 0.0)]
    lost_sigma_per_m: f64,
    /// flatten の代わりに全域相関スキャンマッチで再シード
    /// (`BeliefConfig::global_match` — belief/viterbi のみ)。ロスト中 observe の
    /// 全域走査 (津田沼で最大 ~20 s) を「マッチ 1 回 (~0.5 s) + 疎な top-K
    /// 候補集合」に置き換える。
    #[arg(long)]
    global_match: bool,

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
    /// 診断: 真値仮説の周辺重み / 最大重みとその位置 (全地図のみ)。
    fn probe_weight(&self, p: PoseView) -> (f64, f64, Option<PoseView>) {
        match self {
            Est::Win(_) => (f64::NAN, f64::NAN, None),
            Est::Whole(b) => b.probe_weight(p),
        }
    }
    /// 解除の probation (全地図のみ — 窓つき系は未対応で常に確定解除)。
    fn in_probation(&self) -> bool {
        match self {
            Est::Win(_) => false,
            Est::Whole(b) => b.in_probation(),
        }
    }
    fn fail_probation(&mut self) {
        if let Est::Whole(b) = self {
            b.fail_probation();
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

/// 前方 ±half_rad のビーム最小レンジ [m] (復帰走行の安全ゲート用)。
fn min_range_ahead(scan: &vi_lib::msg::LaserScan, half_rad: f64, max_r: f64) -> f64 {
    let mut m = max_r;
    for (i, &r) in scan.ranges.iter().enumerate() {
        if !(r.is_finite() && r > 0.0 && r <= max_r) {
            continue;
        }
        if wrap_rad(scan.angle_min + scan.angle_increment * i as f64).abs() <= half_rad {
            m = m.min(r);
        }
    }
    m
}


#[allow(clippy::too_many_arguments)]
fn simulate(
    // 能動的再定位が判別場を **in place** で張り替える (実ノードのキャッシュ
    // 差し替えと同じ「一度に 1 枚」規律 — 第 2 フィールドは RAM に載らない)
    // ため &mut。run の出口で必ず主ゴールの場に戻す。
    vi: &mut ValueIterator,
    grid: &OccupancyGrid,
    native: &OccupancyGrid,
    // native の三値版 (-1 unknown 保存) — belief の尤度場専用。世界 (レイ
    // キャスト) は unknown = 障害物の native のまま。
    native_tri: &OccupancyGrid,
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
            // 幾何 (belief 格子) は VI と同じスケール後の grid、尤度場だけ native
            // の三値版 (実ノードは ROS の -1 入り地図をそのまま受けるので、
            // これが実構成)。窓つきの GridLocalizer は native 一本。
            let mut b = Belief::new(grid, N_THETA, native_tri, wbc);
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
    // lost_creep の回頭コミット (前 tick の回頭成分 — 前進 / pose 復帰で 0)。
    let mut creep_rot = 0.0f64;
    // ロスト解除からの経過 tick (安全ゲート・飢餓棄却のスコープ)。初回 relock
    // で切れる recovering と違い、後続の再ロスト解除にも毎回窓が開く。
    // usize::MAX = まだ一度もロストしていない (base 走行はゲート対象外)。
    let mut post_release = usize::MAX;
    let mut was_none = false;
    /// 解除後、安全ゲートと飢餓棄却が有効な窓 [tick] (60 s)。
    const RELEASE_GUARD_TICKS: usize = 600;
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
    /// 復帰走行 (probation / re-lock 前) の前方安全距離 [m]。truth 走行が保つ壁
    /// クリアランス (最小 0.45 m) を邪魔しない範囲で、誤 belief の突進 (K2 の
    /// 0.15 m 衝突) を止める。
    const RECOVERY_GUARD_M: f64 = 0.5;
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
        // 解除後窓の追跡 (None → Some 遷移で開く)。
        if loc.is_some() {
            if est.is_none() {
                was_none = true;
            } else if was_none {
                was_none = false;
                post_release = 0;
            } else if post_release != usize::MAX {
                post_release = post_release.saturating_add(1);
            }
        }
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
                // 診断: 真値仮説の生死 (相対枝刈りで 0 = 死、以後復活不能)。
                if t % 25 == 0 {
                    if let Some(l) = &loc {
                        let (tw, mw, win) = l.probe_weight(PoseView { x, y, yaw_rad: yaw });
                        let (wx, wy, wd) = win
                            .map(|p| (p.x, p.y, (p.x - x).hypot(p.y - y)))
                            .unwrap_or((f64::NAN, f64::NAN, f64::NAN));
                        eprintln!(
                            "  [truth] t={t} w={tw:.2e} max={mw:.2e} ratio={:.1e} win=({wx:.0},{wy:.0}) d={wd:.0}",
                            if mw > 0.0 { tw / mw } else { f64::NAN },
                        );
                    }
                }
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
            // probation 較正/診断: 解除後 (仮・誤とも) の瞬時 quality と真値誤差。
            if r.relock_tick.is_none() && t % 25 == 0 {
                if let (Some(e), Some(l)) = (est, &loc) {
                    eprintln!(
                        "  [prob] t={t} q={:.2} err={:.1} prob={}",
                        l.quality(),
                        ((e.x - x).powi(2) + (e.y - y).powi(2)).sqrt(),
                        l.in_probation(),
                    );
                }
            }
        }

        // 真値スキャン (creep / 復帰走行ゲート / observe で共有 — tick に 1 回)。
        let scan = (loc.is_some() && mode != Mode::Dead).then(|| {
            cast_scan(native, PoseView { x, y, yaw_rad: yaw }, args.scan_beams, args.scan_range)
        });

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
            creep_rot = 0.0;
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
            // 復帰走行の安全ゲート (実機の反射層): 姿勢がまだ検証中 — probation
            // 中か、lost 検出後まだ re-lock していない — の間は、前方 ±30° の
            // スキャンが近いのに前進する指令を握り潰す。cmd=None は既存の飢餓
            // 経路に落ち、probation なら 50 tick で失敗 → 再拡散 (K2: 誤 belief
            // の復帰走行で壁 0.15 m まで詰めて衝突、の対策)。誘拐前の base 走行
            // と検出前の盲走行には触れない — 検出のダイナミクスを変えない。
            let verifying = loc.as_ref().map(|l| l.in_probation()).unwrap_or(false)
                || (recovering && r.detect_tick.is_some())
                || post_release < RELEASE_GUARD_TICKS;
            if let (Some((fw, rot)), true, Some(s)) = (cmd, verifying, scan.as_ref()) {
                if fw > 0.0
                    && min_range_ahead(s, 30f64.to_radians(), args.scan_range) < RECOVERY_GUARD_M
                {
                    if starve == 0 {
                        eprintln!("  [guard] t={t} 前方障害物 — 前進抑止 (検証中)");
                    }
                    // 前進成分だけ殺す — 方策の回頭は通す。正直な近接旋回まで
                    // 全停止すると飢餓 → 棄却 → 再ロストの悪循環になる。回頭も
                    // 無い = 壁へ真っ直ぐ、だけを停止 (飢餓経路 = 誤姿勢の証拠)。
                    cmd = if rot != 0.0 { Some((0.0, rot)) } else { None };
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
                            // 既に居る (< 1 m) 判別点は除く — 到着直後の
                            // 引き直しが同じ点を選んで凍結スラッシュしない。
                            if let Some(&(mp, _)) = hyps.first() {
                                if let Some((tx, ty)) = targets
                                    .iter()
                                    .copied()
                                    .filter(|&(tx, ty)| (tx - mp.x).hypot(ty - mp.y) > 1.0)
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
                    let (fw, rot) = match scan.as_ref() {
                        Some(s) => vi_lib::ctrl::lost_creep(s, preferred, args.scan_range, creep_rot),
                        None => (0.0, 0.0),
                    };
                    creep_rot = rot;
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
                    // probation 中の方策飢餓は「誤姿勢で方策が引けない」という
                    // 外部証拠 — run を殺さず probation を落として lost へ戻す
                    // (誤解除先が非 free に落ちるケースは走行検証まで届かない)。
                    // 復帰中 (re-lock 前) の全地図 belief も同じ扱い: probation
                    // を完走した誤解除が安全ゲートで詰む (津田沼 K2 で実測 —
                    // エイリアスの quality はバーの上に留まり得る) のは同じく
                    // 誤姿勢の外部証拠なので、死なずに再マッチへ回す。
                    let reject = loc.as_ref().map(|l| l.in_probation()).unwrap_or(false)
                        || ((recovering || post_release < RELEASE_GUARD_TICKS)
                            && matches!(loc.as_ref(), Some(Est::Whole(_))));
                    if reject {
                        if let Some(l) = &mut loc {
                            eprintln!("  [prob] t={t} 方策飢餓 → 解除を棄却して lost へ");
                            l.fail_probation();
                        }
                        starve = 0;
                    } else {
                        r.starved = true;
                        r.ticks += 1;
                        break;
                    }
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
                // 判別点に到着 — 凍結も解き、次の tick に**今の** belief から
                // 次の判別点を引き直す。到着後に凍結を保つ旧挙動は、その lost
                // エピソードの残りを盲目の乱歩にしていた (津田沼 K3/K4 で実測:
                // frozen=true / pref=None のまま 180 m 這って収束せず — 1 点
                // 見ただけでは割れない多峰は連続で見に行かないと割れない)。
                reloc_delta = None;
                reloc_frozen = None;
            }
        }

        if let Some(l) = &mut loc {
            let t0 = Instant::now();
            if let Some((v, w_deg)) = moved {
                l.predict(v, w_deg, args.tick_s);
            }
            if let Some(s) = scan.as_ref() {
                l.observe(s);
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
    // belief の尤度場用 (unknown = -1 を保存 — 実ノードが受ける ROS 地図と同じ)。
    let (tocc, _, _) = pgm::build_occupancy_tri(&map, 1);
    let native_tri = OccupancyGrid { data: tocc, ..native.clone() };

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
        probation_obs: args.probation,
        probation_min_q: args.probation_min_q,
        lost_sigma_per_m: args.lost_sigma_per_m,
        global_match: args.global_match,
        ..WholeBeliefConfig::default()
    };
    {
        let probe = GridLocalizer::new(&native, N_THETA, bc);
        let whole = Belief::new(&grid, N_THETA, &native_tri, wbc);
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
                    &native_tri,
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
