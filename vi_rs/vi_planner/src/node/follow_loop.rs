//! 1 ゴールぶんの追従ループ (ゴールごとに専用スレッドで回る)。
//!
//! `core::follow` の判断器 (greedy / DWA / MPPI) が「1 tick でどの行動を出すか」
//! を決めるのに対し、ここはそれを回す**制御ループ**そのもの: 窓の移動 → スキャン
//! 注入 → 予算内の再反復 → 行動の配信、を `control_frequency` で繰り返す。
//!
//! [`run_follow`] が 1 回きりの追従 (`follow_path` = BT 構成)、[`run_goal`] が
//! それを「失敗したら場を落ち着かせて投げ直す」で包んだもの (standalone)、
//! [`run_settle`] が投げ直しの合間に**止まったまま場を更新する**待ち。

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use vi_lib::bridge::PoseView;

use vi_planner::core::{
    lock, mode_count, quality_shift, spread_m, try_lock, value_grid_on, Decision, PlanError,
};

use super::handles::FollowCtx;
use super::msg::{poses_to_path, ros_grid_from, stop_cmd};

/// 制御ループのチューニング (Params から導出)。
#[derive(Clone, Copy)]
pub struct FollowTuning {
    pub period: Duration,
    pub refine_budget: Duration,
    /// pose 欠落 / 方策なしをこの連続 tick 数で追従失敗とみなす。
    pub failure_ticks_limit: u32,
    /// belief が多峰のとき QMDP (`decide_qmdp`) で行動を選ぶ (単峰は従来どおり)。
    pub qmdp: bool,
    /// σ に応じた壁際マージンの膨張が有効か (`PlanConfig::sigma_margin_gain > 0`)。
    /// 立っていると QMDP が off でも上位仮説を取る (σ の測定に要る)。
    pub sigma_margin: bool,
    /// スキャン注入の品質ゲート ([`quality_shift`] の gate)。0 で無効。
    pub scan_quality_gate: f64,
    /// ロスト中に判別点へ走る能動的再定位 (`prepare_reloc_goal` + QMDP)。
    pub active_reloc: bool,
    /// 能動的再定位を諦めて通常の停止待ちに戻すまでの tick 数。
    pub reloc_ticks_limit: u32,
}

/// QMDP に渡す belief 仮説の上限。ヒストグラムの上位セルだけで質量の大半を
/// 覆える (それ以下は veto 判定も動かさない微小仮説)。
const QMDP_TOP_K: usize = 64;

/// 核のロックをこの tick 数**連続で**取れなかったら停止指令を出す。1〜2 tick は
/// 同一ゴールのロールアウト (BT の 1Hz リプラン) や掃きスレッドとの競合なので
/// 止めない。10 Hz なら 3 tick = 300 ms。
const BUSY_TICKS_BEFORE_STOP: u32 = 3;

/// 能動的再定位中の近接停止 [m]。姿勢が無く local_penalty を置けないので、
/// 生の最近接レンジで守る。
// ponytail: 定数 1 個 — 機体寸法に合わせるならパラメータへ昇格。
const RELOC_STOP_RANGE: f64 = 0.35;

/// 能動的再定位 (`active_reloc`) の段階 (追従ループのロスト分岐が使う)。
enum Reloc {
    /// ロストしていない / まだ再定位場を解いていない。
    Idle,
    /// 再定位場の上を QMDP で走行中 (このときキャッシュ = 再定位場)。
    Driving,
    /// この構成では使えない (compact 等) — このゴールでは以後試さない。
    GaveUp,
}

/// QMDP の発火条件で「別の峰」とみなす最小間隔 [m] ([`mode_count`])。
/// 収束した単峰 belief の広がり (数セル) より十分大きく、実際に迷う距離
/// (廊下 1 本ぶん) より小さい値。
// ponytail: 定数。地図の粒度で調整が要るならパラメータへ昇格。
const QMDP_MIN_SEP_M: f64 = 0.5;

pub enum Outcome {
    Reached,
    Preempted,
    Failed(String),
}

/// 追従の 1 tick の様子。アクションごとに Feedback の形が違うので、追従ループ
/// 自体はメッセージ型を知らずにこれを渡す (`follow_path` は距離と速度だけ、
/// `navigate_to_pose` は姿勢と経過時間も要る)。
pub struct FollowProgress {
    pub pose: PoseView,
    /// ゴールまでの XY 距離。ゴール未設定なら None。
    pub distance_remaining: Option<f64>,
    /// この tick に出した前進速度 [m/s] (止めた tick は 0)。
    pub speed: f32,
}

/// 投げ直しのチューニング (`standalone` のときだけ効く)。
#[derive(Clone, Copy)]
pub struct RetryTuning {
    /// 追従が失敗したときに投げ直す上限。負で無制限。
    pub limit: i64,
    /// 投げ直す前に、その場で場を落ち着かせる時間。0 で即座に投げ直す。
    pub settle: Duration,
}

/// 1 ゴールぶんの追従ループ。
///
/// solve 中だけロックを保持し、制御ループは **tick ごとに取得・解放**する
/// (compute_path_to_pose を待たせないため; main.rs 冒頭のロック規律を参照)。
pub fn run_follow(
    ctx: &FollowCtx,
    goal: PoseView,
    cancel: &AtomicBool,
    report: &dyn Fn(&FollowProgress),
) -> Outcome {
    let FollowCtx { core, latest_pose, localizer, scan_queue, cmd_pub, plan_pub, viz, tuning } =
        *ctx;
    // ── 1. 価値関数の用意 (広域側が既に解いていれば何もしない) ──
    {
        let mut core = core.lock().unwrap();
        // solve が要るときだけ止める。BT が同じ経路を再送するたびに 0 速度を
        // 挟むと 1Hz のリプラン周期で走行がぎくしゃくするので、キャッシュ
        // ヒット時は現在の指令を保ったまま次の tick へ引き継ぐ。
        if !core.is_cached_goal(goal) {
            stop_cmd(cmd_pub);
        }

        let t0 = Instant::now();
        let mut last_viz: Option<Instant> = None;
        // 早期走り出し (early_start) の起点。solve のあいだ機体は止まっているので、
        // ここで読んだ姿勢は走り出すときの姿勢そのもの。まだ pose が来ていなければ
        // 打ち切らずに最後まで解く (下の制御ループはどのみち pose を待つ)。
        let from = *latest_pose.lock().unwrap();
        let prepared = core.prepare_goal_with_progress(goal, from, cancel, &mut |vi| {
            let Some(v) = viz else { return };
            if !v.due(&mut last_viz) {
                return;
            }
            let g = value_grid_on(vi, v.threshold_steps);
            let _ = v.vf_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
        });
        match prepared {
            Ok(stats) => {
                if stats.solved_now {
                    eprintln!(
                        // 「追従が用意した場」の意 (広域の plan が解いたぶんと区別する)。
                        // standalone では follow_path ではなく navigate_to_pose の下。
                        "vi_planner: value function {} in {:.2}s (iters={}){} [following]",
                        if stats.adopted { "adopted from prefetch" } else { "solved" },
                        t0.elapsed().as_secs_f64(),
                        stats.iters,
                        // 走り出しを優先した回。ゴールまでの経路は繋がっているが、
                        // そこから外れた領域はまだ未確定で、残りは走りながら背景で
                        // 解き切る (early_start)。
                        if stats.partial {
                            ", enough to start driving — the rest is still being solved in \
                             the background"
                        } else {
                            ""
                        }
                    );
                    // 収束後の最終状態を必ず 1 回配信する。
                    if let Some(v) = viz {
                        if let Some(g) = core.value_grid(v.threshold_steps) {
                            let _ = v.vf_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
                        }
                    }
                }
            }
            Err(PlanError::Cancelled) => return Outcome::Preempted,
            Err(e) => return Outcome::Failed(e.to_string()),
        }

        // 表示用の経路を 1 本だけ出す (スタンドアロンのみ)。場は既に載っている
        // ので、ここは solve ではなくロールアウト 1 回。
        //
        // **失敗しても走行には影響させない。** ロールアウトは貪欲降下なので、
        // 値の起伏が 1 手の進捗より大きい地形で振動して `LoopDetected` になる
        // ことがある (HINT を出す条件そのもの)。追従は方策を 1 手ずつ引くだけ
        // なので、経路が引けなくても走れる。BT 構成ではこの失敗が
        // `ComputePathToPose` の失敗 = ゴールの失敗だった。
        if let Some(pp) = plan_pub {
            let from = *latest_pose.lock().unwrap();
            if let Some(from) = from {
                match core.plan(from, goal, cancel) {
                    Ok((poses, _)) => {
                        let stamp = pp.clock.now().to_sec_nanosec().unwrap_or((0, 0));
                        let _ =
                            pp.path_pub.publish(poses_to_path(&poses, &pp.frame_id, stamp));
                    }
                    Err(e) => eprintln!(
                        "WARN: vi_planner: no path to draw ({e}) — following anyway \
                         (the policy is followed one action at a time, not the path)"
                    ),
                }
            }
        }
    } // ここでロックを解放 — 以降は tick ごとに取り直す。

    // ── 2. 制御ループ ──
    // tick の中では取得した guard が `core` を隠すので、guard を手放したあとに
    // 本体へ触るための別名を先に取っておく。
    let shared = core;
    let mut failure_ticks = 0u32;
    // ロックを連続で取れなかった tick 数 (下の try_lock を参照)。
    let mut busy_ticks = 0u32;
    let mut last_viz: Option<Instant> = None;
    let mut reloc = Reloc::Idle;
    let mut reloc_ticks = 0u32;
    loop {
        let tick_start = Instant::now();
        if cancel.load(Ordering::Relaxed) {
            stop_cmd(cmd_pub);
            return Outcome::Preempted;
        }

        let pose = *latest_pose.lock().unwrap();
        // 能動的再定位から復帰した直後: キャッシュには再定位場が載っている。
        // 下の「goal replaced」プリエンプトと誤認する前に本来のゴールを解き直す
        // (再定位中に本当に別の追従ゴールが来る形は cancel 経由で上で抜けている)。
        if matches!(reloc, Reloc::Driving) {
            if let Some(p) = pose {
                let Some(mut core) = try_lock(core) else {
                    if let Some(rest) = tuning.period.checked_sub(tick_start.elapsed()) {
                        std::thread::sleep(rest);
                    }
                    continue;
                };
                stop_cmd(cmd_pub); // 解き直しの数秒、直前の指令で這わせない
                if !core.is_cached_goal(goal) {
                    eprintln!(
                        "vi_planner: re-localized at ({:.2}, {:.2}) — re-solving the goal \
                         [active_reloc]",
                        p.x, p.y
                    );
                    if let Err(e) = core.prepare_goal(goal, cancel) {
                        return match e {
                            PlanError::Cancelled => Outcome::Preempted,
                            e => Outcome::Failed(e.to_string()),
                        };
                    }
                }
                reloc = Reloc::Idle;
                reloc_ticks = 0;
                failure_ticks = 0;
            }
        }
        let Some(pose) = pose else {
            // ── ロスト (pose なし)。既定は安全停止で受動復帰 (adaptive の expansion
            // resetting / 全地図 belief の一様混合リセット) を待つ。active_reloc なら
            // 仮説を判別する地点の多目標場を解き、QMDP で走って復帰を早める
            // (core::prepare_reloc_goal の doc)。
            let mut acted = false;
            if tuning.active_reloc
                && reloc_ticks < tuning.reloc_ticks_limit
                && !matches!(reloc, Reloc::GaveUp)
            {
                reloc_ticks += 1;
                acted = 'reloc: {
                    // localizer → core の順 (入れ子ロックを作らない)。
                    let (hyps, targets) = {
                        let l = lock(localizer);
                        (l.top_cells(QMDP_TOP_K), l.reloc_targets())
                    };
                    if hyps.len() < 2 {
                        break 'reloc false;
                    }
                    // 近接ガード: 姿勢が無く local_penalty を置けないので、生の
                    // 最近接レンジで守る (スキャンはコールバックで localizer に
                    // 反映済みなので捨ててよい)。
                    let scans = std::mem::take(&mut *scan_queue.lock().unwrap());
                    if scans.last().is_some_and(|s| {
                        s.ranges
                            .iter()
                            .filter(|r| r.is_finite() && **r > 0.0)
                            .fold(f64::INFINITY, |a, &r| a.min(r))
                            < RELOC_STOP_RANGE
                    }) {
                        stop_cmd(cmd_pub);
                        break 'reloc true; // 止まって観測を待つのも再定位のうち
                    }
                    let Some(mut core) = try_lock(core) else { break 'reloc false };
                    if matches!(reloc, Reloc::Idle) {
                        if targets.is_empty() {
                            break 'reloc false;
                        }
                        match core.prepare_reloc_goal(&targets, cancel) {
                            Ok(_) => {
                                eprintln!(
                                    "vi_planner: lost — driving toward {} disambiguation \
                                     target(s) [active_reloc]",
                                    targets.len()
                                );
                                reloc = Reloc::Driving;
                            }
                            Err(PlanError::Cancelled) => break 'reloc false,
                            Err(e) => {
                                // compact 構成など — 受動復帰に任せる。
                                eprintln!("vi_planner: active_reloc unavailable ({e})");
                                reloc = Reloc::GaveUp;
                                break 'reloc false;
                            }
                        }
                    }
                    match core.decide_qmdp(&hyps) {
                        Decision::Action { fw, rot_deg, .. } => {
                            drop(core); // localizer を取る前に手放す (ロック順序)
                            let mut tw = geometry_msgs::msg::Twist::default();
                            tw.linear.x = fw;
                            tw.angular.z = rot_deg.to_radians();
                            let _ = cmd_pub.publish(tw);
                            let mut l = lock(localizer);
                            l.predict(fw, rot_deg, tuning.period.as_secs_f64());
                            *lock(latest_pose) = l.pose();
                            true
                        }
                        // Goal = 判別点に到着 / NoAction — 止まって観測を待つ。
                        _ => {
                            stop_cmd(cmd_pub);
                            true
                        }
                    }
                };
            }
            if acted {
                failure_ticks = 0;
            } else {
                stop_cmd(cmd_pub);
                failure_ticks += 1;
                if failure_ticks >= tuning.failure_ticks_limit {
                    return Outcome::Failed("no robot pose for too long".into());
                }
            }
            if let Some(rest) = tuning.period.checked_sub(tick_start.elapsed()) {
                std::thread::sleep(rest);
            }
            continue;
        };
        // 追従が belief から要るもの 2 つ — QMDP 用の上位仮説と、スキャン注入の
        // 品質ゲート (直近補正の観測一致度 → 減衰段数) — を **1 回のロックで**
        // まとめて取る。core のロックの前に取るので (localizer → core) 入れ子は
        // 作らない。external は仮説なし・quality 1.0 = 従来と同じ挙動。
        let (hyps, scan_shift, sigma) = {
            let l = lock(localizer);
            // σ (マージン膨張の量) も同じ仮説集合から測るので、qmdp が off でも
            // sigma_margin が立っていれば取る。external は空 = σ 0 = 膨張なし。
            let cells = if tuning.qmdp || tuning.sigma_margin {
                l.top_cells(QMDP_TOP_K)
            } else {
                Vec::new()
            };
            let sigma = if tuning.sigma_margin { spread_m(&cells) } else { 0.0 };
            (
                if tuning.qmdp { cells } else { Vec::new() },
                quality_shift(l.quality(), tuning.scan_quality_gate),
                sigma,
            )
        };

        // ── ロックを取るのはこのブロックだけ ──
        //
        // `lock()` ではなく `try_lock()` にしてある。広域側が **別ゴール** の
        // 計画要求を受けると、その solve は数秒〜数十秒ロックを握り続ける。
        // ここでブロックすると、その間ずっと直前の速度指令が出たままロボットが
        // 走り続けることになる (velocity_smoother のタイムアウト任せにしない)。
        let Some(mut core) = try_lock(core) else {
            // 1〜2 tick 取れないのは同一ゴールのロールアウト待ち (BT の 1Hz
            // リプランはキャッシュヒットでも rollout + densify をロック内で回す)。
            // ここで毎回止めると、取り除いたはずの「1 秒ごとに 0 速度が挟まる」
            // 挙動が戻ってしまうので、直前の指令を保ったまま次の tick を待つ。
            // 連続で取れない = 本当に長い solve が走っているので、そのときだけ止める。
            busy_ticks += 1;
            if busy_ticks >= BUSY_TICKS_BEFORE_STOP {
                stop_cmd(cmd_pub);
            }
            if let Some(rest) = tuning.period.checked_sub(tick_start.elapsed()) {
                std::thread::sleep(rest);
            }
            continue;
        };
        busy_ticks = 0;

        // scan の取り込みはロックを取れた tick でだけ行う (取れなかった tick で
        // 捨てると、待っている間のスキャンが丸ごと失われる)。ビジーが続くと
        // キューは 10 件の上限に張り付き、次に取れた tick でまとめて同一 pose の
        // もとに注入される — スキャンコールバック側の上限はこのための蓋。
        let scans = std::mem::take(&mut *scan_queue.lock().unwrap());

        let (decision, dist, window_grid) = {
            // 広域側が別ゴールを解いてキャッシュを差し替えていたら、その方策で
            // 走らせるわけにはいかない。既存のプリエンプトと同じ扱いで抜ける。
            if !core.is_cached_goal(goal) {
                stop_cmd(cmd_pub);
                eprintln!("vi_planner: follow preempted — the cached goal was replaced");
                return Outcome::Preempted;
            }
            core.set_window(pose);
            for scan in &scans {
                core.observe_scan_gated(scan, pose, scan_shift);
            }
            // 自己位置が曖昧なほど壁際のマージンを広げる (文献 4·2·2)。注入の後に
            // 置く — 帯は地図の壁から起こすので、スキャンの塗り直しと打ち消し合わない。
            core.inflate_by_sigma(sigma, pose);
            core.refine_for(tuning.refine_budget);

            // 可視化グリッドの作成はロック内 (states を読む) / 配信は外。
            let mut window_grid = None;
            if let Some(v) = viz {
                if v.due(&mut last_viz) {
                    window_grid = core.window_value_grid(pose, v.threshold_steps);
                }
            }
            // 多峰 belief は QMDP — どの仮説でも悪くない行動を選び、有意な仮説が
            // 衝突と言う行動しか無ければ止まる。単峰は従来の decide
            // (follow_controller 経由) と一致するので点推定に退避。
            //
            // 多峰性は**セル数ではなく峰の数**で測る: 全地図 belief のアクティブ
            // 集合は収束していても数千セルあるので、`hyps.len() >= 2` は常に真に
            // なり follow_controller が一度も動かない (tb3 デモで実測、
            // `mode_count` の doc 参照)。
            let decision = if mode_count(&hyps, QMDP_MIN_SEP_M) >= 2 {
                core.decide_qmdp(&hyps)
            } else {
                core.decide(pose)
            };
            (decision, core.goal_distance(pose.x, pose.y), window_grid)
        };
        // ここで手放す。スコープ末尾まで持つと sleep の間もロックを握ったままに
        // なり、広域側の計画要求がほぼ通らなくなる。
        drop(core);

        // 配信はロックの外で (可視化 1 枚は 100 万セル級になり得る)。
        if let (Some(v), Some(g)) = (viz, window_grid) {
            let _ = v.win_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
        }

        // 制御ループは tick ごとにロックを手放すので、プリエンプトした新しい
        // 追従スレッドと一瞬だけ並走し得る (旧 vi_local_planner は追従中ずっと
        // ロックを握っていたので起こらなかった)。古いループの指令が新しい
        // ループの指令を上書きしないよう、publish の直前でもう一度観測する。
        if cancel.load(Ordering::Relaxed) {
            stop_cmd(cmd_pub);
            return Outcome::Preempted;
        }

        let mut speed = 0.0f32;
        match decision {
            Decision::Goal => {
                stop_cmd(cmd_pub);
                return Outcome::Reached;
            }
            Decision::Action { fw, rot_deg, .. } => {
                // 本家 ViNode::decision: delta をそのまま速度指令に。
                let mut tw = geometry_msgs::msg::Twist::default();
                tw.linear.x = fw;
                tw.angular.z = rot_deg.to_radians();
                let _ = cmd_pub.publish(tw);
                // 自分が出した指令がそのまま推定器の動作モデル (external では
                // no-op)。dt は制御周期 — 実際の適用時間との差は motion ノイズが
                // 吸収する。停止指令は動きゼロなので predict しない。代入は
                // 無条件 — belief がロストしたら latest_pose ごと消え、次 tick
                // から「pose なし」の安全停止に入る。
                {
                    let mut l = lock(localizer);
                    l.predict(fw, rot_deg, tuning.period.as_secs_f64());
                    *lock(latest_pose) = l.pose();
                }
                speed = fw as f32;
                failure_ticks = 0;
            }
            Decision::NoAction => {
                stop_cmd(cmd_pub);
                failure_ticks += 1;
            }
        }

        report(&FollowProgress { pose, distance_remaining: dist, speed });

        if failure_ticks >= tuning.failure_ticks_limit {
            stop_cmd(cmd_pub);
            // まだ解き終わっていない場 (early_start) だったなら捨てる。方策が引けないのは
            // 「経路の外の未確定領域に入った」形で起き得るが、キャッシュを
            // 残したままだと BT が投げ直すたびに同じ場で同じ失敗を繰り返す
            // (prepare_goal はキャッシュヒットで solve に入らない)。捨てておけば
            // 次の要求が最後まで解き直す。収束済みの場なら何もしない。
            if lock(shared).discard_partial() {
                eprintln!(
                    "vi_planner: dropped the not-yet-finished value function (early_start) after \
                     {failure_ticks} ticks without an action; the next request solves it \
                     to convergence"
                );
            }
            return Outcome::Failed("no applicable action for too long".into());
        }
        if let Some(rest) = tuning.period.checked_sub(tick_start.elapsed()) {
            std::thread::sleep(rest);
        }
    }
}

/// 止まったまま場を更新し続ける「待ち」。投げ直しの前に挟む。
///
/// **Nav2 の BT の `Wait` はこれができない。** あちらが待っている間は
/// `follow_path` が走っていないので `observe_scan` も `refine_for` も呼ばれず、
/// つまり `set_local_cost` の penalty 半減 — 一度«通れない»と塗った場所が
/// «やっぱり通れる»に戻る唯一の経路 — が 1 段も進まない。待ち時間を延ばしても
/// 場が同じままなら、投げ直しは同じ失敗を同じ場所で繰り返すだけになる
/// (2026-08-04 の実機で `no_action_timeout_sec` を 15 秒へ延ばして確認: 粘らず、
/// 1 点飛ばすまでが 2 分に伸びただけだった)。
///
/// ここでは 0 速度を出しながら制御ループと同じ更新 (窓の移動 → スキャン注入 →
/// 精密化) を回す。ロックの持ち方も制御ループと同じ `try_lock` で、掃きスレッド
/// からも先読みからも取り上げない。
///
/// 戻り値は「最後まで待てたか」。false は cancel された。
pub fn run_settle(ctx: &FollowCtx, cancel: &AtomicBool, dur: Duration) -> bool {
    let FollowCtx { core, latest_pose, localizer, scan_queue, cmd_pub, tuning, .. } = *ctx;
    let deadline = Instant::now() + dur;
    while Instant::now() < deadline {
        let tick_start = Instant::now();
        if cancel.load(Ordering::Relaxed) {
            return false;
        }
        stop_cmd(cmd_pub);

        let pose = *latest_pose.lock().unwrap();
        if let Some(pose) = pose {
            if let Some(mut c) = try_lock(core) {
                let scan_shift =
                    quality_shift(lock(localizer).quality(), tuning.scan_quality_gate);
                let scans = std::mem::take(&mut *scan_queue.lock().unwrap());
                c.set_window(pose);
                for scan in &scans {
                    c.observe_scan_gated(scan, pose, scan_shift);
                }
                c.refine_for(tuning.refine_budget);
            }
        }

        if let Some(rest) = tuning.period.checked_sub(tick_start.elapsed()) {
            std::thread::sleep(rest);
        }
    }
    true
}

/// スタンドアロンの 1 ゴール = `navigate_to_pose` の中身。
///
/// [`run_follow`] を「失敗したら場を落ち着かせて投げ直す」で包んだだけだが、
/// これが Nav2 の BT (`RecoveryNode` + `RoundRobin` の Spin / Wait / BackUp) の
/// 置き換えになっている。BT との違いは 2 つ:
///
///   * リカバリが**必ず動く**。`Spin` / `BackUp` は `local_costmap/costmap_raw`
///     を待つが、VI 構成にコストマップは無いので必ず失敗していた。
///   * 待つ間に**場が動く** ([`run_settle`])。BT の `Wait` は止まるだけだった。
///
/// `retries` は Feedback の `number_of_recoveries` 用に外から覗くための共有カウンタ。
pub fn run_goal(
    ctx: &FollowCtx,
    goal: PoseView,
    cancel: &AtomicBool,
    retry: RetryTuning,
    retries: &AtomicU64,
    report: &dyn Fn(&FollowProgress),
) -> Outcome {
    retries.store(0, Ordering::Relaxed);
    loop {
        match run_follow(ctx, goal, cancel, report) {
            Outcome::Failed(reason) => {
                let done = retries.load(Ordering::Relaxed);
                if retry.limit >= 0 && done as i64 >= retry.limit {
                    return Outcome::Failed(format!("{reason} (after {done} retries)"));
                }
                retries.store(done + 1, Ordering::Relaxed);
                eprintln!(
                    "vi_planner: follow failed ({reason}); settling for {:.1}s, then retry {}{}",
                    retry.settle.as_secs_f64(),
                    done + 1,
                    if retry.limit >= 0 {
                        format!("/{}", retry.limit)
                    } else {
                        String::new()
                    }
                );
                if !retry.settle.is_zero() && !run_settle(ctx, cancel, retry.settle) {
                    stop_cmd(ctx.cmd_pub);
                    return Outcome::Preempted;
                }
                if cancel.load(Ordering::Relaxed) {
                    stop_cmd(ctx.cmd_pub);
                    return Outcome::Preempted;
                }
            }
            other => return other,
        }
    }
}
