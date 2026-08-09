//! 起動時の重い準備 — /map の待ち受け ([`wait_for_map`]) と、地図 +
//! パラメータからの核の組み立て ([`build_core`])。
//!
//! ここで落とす (= 起動を止める) のは、黙って確保して OOM killer に落とされる
//! より理由を出して止めるほうがよいものだけ (密の価値関数の見積もり)。

use std::path::PathBuf;
use std::sync::atomic::AtomicU64;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{anyhow, Result};

use vi_lib::bridge::{
    downsample_occupancy, downsample_occupancy_optimistic, occupancy_view_to_vi_grid,
    OccupancyGridView,
};
use vi_lib::solvers::U64Solver;
use vi_lib::Action;

use vi_planner::core::{
    AdaptiveLocalizer, Belief, BeliefConfig, BuildParams, FollowKind, GridLocalizer, PlanConfig,
    PlannerCore, Prefetcher, WholeMapBeliefConfig,
};

use rclrs::*;

use super::handles::Loc;
use super::params::Params;

// ── 正しい値が 1 つしかない定数 ─────────────────────────────────────────────
// どれも起動パラメータだったもの。launch から変える判断材料が無く (「掃きの
// チャンクを何セルにすべきか」に答えられるユーザーはいない)、変えて良くなる場面
// も無かったので、理由と一緒にここへ落としてある。昇格させたくなったら
// node::params へ戻すだけ。

/// solve 総イテレーション上限 (発散ガード)。終了条件は収束なので実質無限。
const MAX_SOLVE_ITER: u32 = 1_000_000;
/// cancel を観測する刻み [イテレーション]。プリエンプトの粒度 (密経路のみ)。
const SOLVE_CHUNK: u32 = 64;
/// start が方策なしセルのとき近傍を探す範囲 [m]。
const START_TOLERANCE_M: f64 = 0.5;
/// 現在セルに方策が無いとき近傍から行動を借りる範囲 [m]。
const ACTION_TOLERANCE_M: f64 = 0.2;
/// compact パッチの寸法スラック [セル] / 修復タイルの interior の 1 辺 [セル]
/// (効き方は `core::BuildParams` の doc)。
const PATCH_SLACK_CELLS: i32 = 2;
const REPAIR_INTERIOR_CELLS: i32 = 16;
/// 進行中の先読みを待つときの観測間隔 [ms] (cancel を見る刻みでもある)。
const PREFETCH_POLL_MS: u64 = 50;
/// DWA の (v, ω) 候補数と MPPI の softmax 温度。実測 decide は DWA ~30 µs /
/// MPPI ~0.2 ms で、10 Hz の 40 ms 予算からは 3 桁遠い = 詰める理由が無い。
const DWA_N_V: usize = 7;
const DWA_N_W: usize = 11;
const MPPI_LAMBDA: f64 = 1.0;
/// MPPI の制御ノイズ σ。0 = 行動集合から自動 (σ_v は速度幅の 1/4、σ_ω は上限の 1/4)。
const MPPI_SIGMA_AUTO: f64 = 0.0;
/// 投げ直しの前に、止まったままスキャンを取り込んで場を精密化する時間 [s]
/// (standalone の `run_settle` — BT の `Wait` と違って場が実際に動く)。
pub const GOAL_RETRY_SETTLE_SEC: f64 = 3.0;
/// 能動的再定位を諦めて通常の停止待ちに戻すまでの時間 [s]。
pub const RELOC_TIMEOUT_SEC: f64 = 30.0;

/// このマシンで**いま**確保できるメモリ [B] (`/proc/meminfo` の `MemAvailable`)。
/// 読めなければ `None` = 判定そのものを見送る (推測で起動を止めない)。
fn mem_available() -> Option<u64> {
    let s = std::fs::read_to_string("/proc/meminfo").ok()?;
    let kb: u64 = s
        .lines()
        .find(|l| l.starts_with("MemAvailable:"))?
        .split_whitespace()
        .nth(1)?
        .parse()
        .ok()?;
    Some(kb * 1024)
}

/// /map (transient_local) を最初の 1 通が来るまで待つ。
pub fn wait_for_map(
    node: &Node,
    executor: &mut Executor,
    wait_sec: i64,
) -> Result<nav_msgs::msg::OccupancyGrid> {
    use std::sync::mpsc::sync_channel;

    let (tx, rx) = sync_channel::<nav_msgs::msg::OccupancyGrid>(1);

    let _sub = node.create_subscription::<nav_msgs::msg::OccupancyGrid, _>(
        "map".transient_local().reliable().keep_last(1),
        move |msg: nav_msgs::msg::OccupancyGrid| {
            let _ = tx.try_send(msg);
        },
    )?;

    let deadline = Instant::now() + Duration::from_secs(wait_sec as u64);
    loop {
        if let Ok(msg) = rx.try_recv() {
            return Ok(msg);
        }
        if Instant::now() > deadline {
            return Err(anyhow!("map not received within {} seconds", wait_sec));
        }
        executor.spin(SpinOptions::default().timeout(Duration::from_millis(100)));
    }
}

/// 広域・狭域が共有する唯一の価値関数 ([`PlannerCore`])、自己位置推定
/// ([`Loc`])、ウェイポイント先読み ([`Prefetcher`]) を地図から組む。
pub fn build_core(
    params: &Params,
    solver: U64Solver,
    map_msg: &nav_msgs::msg::OccupancyGrid,
) -> Result<(PlannerCore, Loc, Option<Prefetcher>)> {
    let grid_view = OccupancyGridView {
        width: map_msg.info.width,
        height: map_msg.info.height,
        resolution: map_msg.info.resolution as f64,
        origin_x: map_msg.info.origin.position.x,
        origin_y: map_msg.info.origin.position.y,
        data: &map_msg.data[..],
    };
    // プランナ内部の作業解像度。密ソルバは状態配列 (56 B/state) を全域ぶん確保するので、
    // 広域地図では map_scale がそのままメモリを決める。compact ソルバなら確定出力
    // 12 B/state を sink (mmap) に逃がし、追従はロボット近傍のパッチだけを密に起こす。
    let binary_grid = occupancy_view_to_vi_grid(&grid_view, params.unknown_as_obstacle);
    let vi_grid = match params.downsample_policy.as_str() {
        "optimistic" => downsample_occupancy_optimistic(&binary_grid, params.map_scale as i32),
        "conservative" => downsample_occupancy(&binary_grid, params.map_scale as i32),
        other => {
            return Err(anyhow!(
                "downsample_policy must be \"conservative\" or \"optimistic\", got {other:?}"
            ))
        }
    };
    // 自己位置推定。窓つき (grid / adaptive / adaptive_viterbi) は native 解像度の
    // 占有格子から belief も尤度場も起こす。全地図 (belief / viterbi) は
    // **幾何と free マスクはプランナと同じ vi_grid** (map_scale 適用後) から、
    // **尤度場だけは native 解像度の binary_grid** から起こす — スキャン端点の
    // 評価は格子を粗くすると壊れるため。どちらも binary_grid を使い切ってから捨てる。
    let localizer = match params.localizer.as_str() {
        kind @ ("grid" | "adaptive" | "adaptive_viterbi") => {
            let bc = BeliefConfig {
                half_m: params.belief_radius.max(0.5),
                sensor_sigma_m: params.belief_sensor_sigma.max(0.01),
                beam_step: params.belief_beam_step.max(1) as usize,
                max_range_m: params.belief_max_range.max(0.1),
                motion_sigma_xy_m: params.belief_motion_sigma_xy.max(0.0),
                motion_sigma_theta_deg: params.belief_motion_sigma_theta_deg.max(0.0),
                // "adaptive_viterbi" = adaptive の全域レベルを min-plus (MAP) で回す変種。
                viterbi: kind == "adaptive_viterbi",
                ..BeliefConfig::default()
            };
            if kind == "grid" {
                let g = GridLocalizer::new(&binary_grid, params.theta_cell_num as i32, bc);
                eprintln!(
                    "vi_planner: localizer = grid ({}m window @{}m x{} theta, {:.1} MB belief; \
                     seed it via {} — e.g. remap to initialpose)",
                    params.belief_radius * 2.0,
                    binary_grid.resolution,
                    params.theta_cell_num,
                    g.belief_mb(),
                    params.pose_topic
                );
                Loc::Windowed(Box::new(g))
            } else {
                let g = AdaptiveLocalizer::new(&binary_grid, params.theta_cell_num as i32, bc);
                eprintln!(
                    "vi_planner: localizer = {} ({} levels up to whole-map, {:.1} MB \
                     belief; seed via {} or leave unseeded for global init; lost -> pose \
                     withheld until re-localized)",
                    kind,
                    g.num_levels(),
                    g.belief_mb(),
                    params.pose_topic
                );
                Loc::Windowed(Box::new(g))
            }
        }
        kind @ ("belief" | "viterbi") => {
            let bc = WholeMapBeliefConfig {
                sensor_sigma_m: params.belief_sensor_sigma.max(0.01),
                beam_step: params.belief_beam_step.max(1) as usize,
                max_range_m: params.belief_max_range.max(0.1),
                motion_sigma_xy_m: params.belief_motion_sigma_xy.max(0.0),
                motion_sigma_theta_deg: params.belief_motion_sigma_theta_deg.max(0.0),
                // "viterbi" = 同じ belief を min-plus (MAP) 半環で回す変種。
                viterbi: kind == "viterbi",
                ..WholeMapBeliefConfig::default()
            };
            let b = Belief::new(&vi_grid, params.theta_cell_num as i32, &binary_grid, bc);
            eprintln!(
                "vi_planner: localizer = {} (whole-map belief on the planner grid: \
                 {} free cells x{} theta, {:.1} MB; likelihood field from the native map \
                 @{}m; seed via {} or leave unseeded for global init; lost -> pose withheld \
                 until re-localized)",
                kind,
                b.free_cells(),
                params.theta_cell_num,
                b.belief_mb(),
                binary_grid.resolution,
                params.pose_topic
            );
            Loc::Belief(Box::new(b))
        }
        _ => Loc::External(None),
    };
    drop(binary_grid);
    let nstates =
        vi_grid.width as usize * vi_grid.height as usize * params.theta_cell_num as usize;
    let use_compact = solver.caps().out_of_core;
    // 密が実際に確保するのは `states` (State 56 B/state) だけではない。
    // `set_sweep_orders` が掃き順を 6 本ぶん持つ (`[0..3]` 各 total、`[4]` 1.5×total、
    // `[5]` 0.5×total = 合わせて 6.0×total の i32) ので +24 B/state。
    // 19F を map_scale 2 で解いたときの実測は 654.8 MB (states 444.7 + orders 210.1) で、
    // 56 B/state だけで見積もると 4 割以上足りない。compact は確定出力 12 B/state だけ。
    let states_gb = nstates as f64 * 80.0 / 1e9;
    let sink_gb = nstates as f64 * 12.0 / 1e9;
    // 先読みは価値関数を 2 本持つ (走っている点のと、次の点の)。密ならメモリ、
    // compact ならディスクがそのぶん要る。見積もりは常に 2 倍で語ること。
    let fields = if params.waypoint_prefetch { 2.0 } else { 1.0 };
    eprintln!(
        "vi_planner: planner grid {}x{} @{:.3}m (map_scale={}, downsample={}) x{} theta \
         = {} states{}",
        vi_grid.width,
        vi_grid.height,
        vi_grid.resolution,
        params.map_scale,
        params.downsample_policy,
        params.theta_cell_num,
        nstates,
        if use_compact {
            String::new()
        } else {
            format!(", dense states+sweep_orders {states_gb:.2} GB")
        },
    );
    // sink の置き場と、そこを選んだ理由の 1 行はこの中で出る (明示指定 →
    // 空きメモリによる自動退避 → RAM)。密ソルバなら None。
    let sink_dir = compact_sink_dir(params, solver, nstates);
    if params.waypoint_prefetch {
        // 生きる場はちょうど 2 つ (走行中のと、次の) で頭打ちになる。新しい場を
        // 確保する前に必ず古いほうを手放しているため — 走行中の核は solve の前に
        // `cached = None`、先読みは注文を受ける前に採用待ちを捨てる。
        eprintln!(
            "vi_planner: waypoint prefetch on ({} threads) — {} for {:.2} GB \
             (2 x {:.2} GB: the goal being driven to, and the next one)",
            params.waypoint_prefetch_threads.max(1),
            if use_compact { "budget the sink directory" } else { "budget RAM" },
            if use_compact { sink_gb * 2.0 } else { states_gb * 2.0 },
            if use_compact { sink_gb } else { states_gb },
        );
    }
    // 密の見積もりが**いまの空きメモリ**を超えたら起動を止める。ここまで来れば
    // 地図の実寸が分かっているので、launch 側の代理判定 (map_scale > 1 なら密を
    // 禁止) より正確。黙って確保して OOM killer に落とされるより、理由を出して
    // 止めるほうがよい。かつては dense_limit_mb という起動パラメータだったが、
    // 答えは「このマシンの空き」であって固定の MB 数ではない (4GB 機で妥当な値は
    // 64GB 機では邪魔なだけで、しかも正しい数字はユーザーには分からない)。
    // 半分を超えたら警告、全部を超えたら停止。先読みを入れると場が 2 本になるので、
    // 突き合わせるのも 2 本ぶん。
    let need_gb = states_gb * fields;
    if !use_compact {
        // /proc/meminfo が読めない環境では判定を見送る (推測で起動は止めない)。
        if let Some(avail_gb) = mem_available().map(|b| b as f64 / 1e9) {
            let doubled = if params.waypoint_prefetch {
                " x2 value functions (waypoint_prefetch: true)"
            } else {
                ""
            };
            if need_gb > avail_gb {
                return Err(anyhow!(
                    "the dense value function needs {need_gb:.2} GB (states + sweep_orders) for \
                     {nstates} states{doubled}, but only {avail_gb:.2} GB is available on this \
                     machine.\n\
                     Raise map_scale (halving the resolution quarters this), free memory, or \
                     switch to solver: frontier2d_sparse_compact (+ compact_sink_dir), which \
                     keeps only {sink_gb:.2} GB of finalized output and hydrates a small patch \
                     around the robot for following. The local -> global feedback (global_sweep) \
                     works there too — it repairs the sink tile by tile instead of sweeping a \
                     full `states` array."
                ));
            }
            if need_gb > avail_gb * 0.5 {
                eprintln!(
                    "WARN: the dense value function takes {need_gb:.2} GB{doubled} of the \
                     {avail_gb:.2} GB available — over half the machine. Raise map_scale or \
                     switch to solver: frontier2d_sparse_compact if anything else needs to run \
                     here."
                );
            }
        }
    }
    let start_tolerance_cells = (START_TOLERANCE_M / vi_grid.resolution).ceil() as i32;
    let action_tolerance_cells = (ACTION_TOLERANCE_M / vi_grid.resolution).ceil() as i32;

    let build = BuildParams {
        grid: vi_grid,
        actions: params
            .action_list
            .iter()
            .enumerate()
            .map(|(i, (name, fw, rot))| Action::new(name, *fw, *rot, i as i32))
            .collect(),
        theta_cell_num: params.theta_cell_num as i32,
        safety_radius: params.safety_radius,
        safety_radius_penalty: params.safety_radius_penalty as f64,
        goal_margin_radius: params.goal_margin_radius,
        goal_margin_theta: params.goal_margin_theta_deg as i32,
        local_xy_range: params.local_xy_range,
        patch_slack_cells: PATCH_SLACK_CELLS,
        repair_interior_cells: REPAIR_INTERIOR_CELLS,
    };
    // validate() 済みなので必ず解ける。
    let follow_kind = FollowKind::from_name(&params.follow_controller)
        .ok_or_else(|| anyhow!("unknown follow_controller: {}", params.follow_controller))?;
    if follow_kind == FollowKind::Dwa {
        eprintln!(
            "vi_planner: follow controller = dwa (continuous; horizon {:.2} s, {}x{} candidates, \
             greedy fallback)",
            params.dwa_horizon_s, DWA_N_V, DWA_N_W
        );
    } else if follow_kind == FollowKind::Mppi {
        eprintln!(
            "vi_planner: follow controller = mppi (continuous; horizon {:.2} s, {} samples, \
             lambda {:.2}, greedy fallback)",
            params.dwa_horizon_s, params.mppi_samples, MPPI_LAMBDA
        );
    }
    let cfg = PlanConfig {
        solver,
        max_solve_iter: MAX_SOLVE_ITER,
        solve_chunk: SOLVE_CHUNK,
        goal_tolerance_xy: params.goal_tolerance_xy,
        goal_tolerance_deg: params.goal_tolerance_deg,
        max_rollout_steps: params.max_rollout_steps.max(1) as usize,
        start_tolerance_cells,
        path_spacing: params.path_spacing,
        action_tolerance_cells,
        follow_controller: follow_kind,
        sigma_margin_gain: params.sigma_margin_gain,
        map_clear_from_scan: params.map_clear_from_scan,
        dwa_tick_s: 1.0 / params.control_frequency,
        dwa_horizon_s: params.dwa_horizon_s,
        dwa_n_v: DWA_N_V,
        dwa_n_w: DWA_N_W,
        dwa_lethal_penalty: params.dwa_lethal_penalty.max(0.0),
        mppi_samples: params.mppi_samples.max(2) as usize,
        mppi_lambda: MPPI_LAMBDA,
        mppi_sigma_v: MPPI_SIGMA_AUTO,
        mppi_sigma_w_deg: MPPI_SIGMA_AUTO,
        compact_sink_dir: sink_dir,
        // 先読みを入れると場が 2 つ同時に生きるので、compact の確定出力は solve
        // ごとに使い捨てのディレクトリ (<compact_sink_dir>/gen<N>) へ分ける。
        // 固定ファイル名のままだと後から解くほうが先の場のファイルを truncate で
        // 潰す (mmap したまま長さが変わる = ゼロを読むか SIGBUS)。カウンタは
        // 先読み側の核と共有すること。
        compact_sink_gen: params.waypoint_prefetch.then(|| Arc::new(AtomicU64::new(0))),
        vi_threads: params.vi_threads.max(0) as usize,
        prefetch_poll_ms: PREFETCH_POLL_MS,
        // 早期走り出しは背景の解き切りとセット (掃きスレッドの起動側と同じ判断)。
        global_sweep: params.global_sweep || params.early_start,
        early_start: params.early_start,
    };
    if params.early_start && !params.global_sweep {
        eprintln!(
            "vi_planner: global_sweep was off and has been turned on — early_start needs it to \
             finish the field while the robot drives"
        );
    }
    if params.early_start {
        eprintln!(
            "vi_planner: early start on — the robot leaves as soon as the policy reaches the \
             goal from where it stands; the solve is NOT abandoned there, the rest of the field \
             is finished in the background while driving (dense: the global sweep; compact: the \
             tile repair, seeded with the whole map) and it is logged again once converged"
        );
    }

    // ウェイポイントの先読み (waypoint_prefetch)。予備の核を 1 つ専用スレッドに
    // 持たせ、いまの点へ走っている間に次の点を解かせる。走行中の核のロックは
    // 取らないので、追従ループ (10Hz / try_lock) の邪魔はしない。
    let prefetch = params.waypoint_prefetch.then(|| {
        let cfg = PlanConfig {
            // 先読みの場に狭域の書き込みは無い = 伝播させる仕事が無いので、
            // 修復タイル (数 MB + 遷移表の再計算) を作らせない。
            global_sweep: false,
            // 追従の 40ms/tick を削らないよう既定 1 に絞る (compact のみ有効。
            // 密は VI_THREADS がプロセスで 1 つなので分けられない)。
            vi_threads: params.waypoint_prefetch_threads.max(1) as usize,
            // 先読みは最後まで解く。早期走り出しの起点は「いまの機体の姿勢」だが、
            // 次の点に着く頃には機体はそこにいない (途中で返した場は使えない)。
            // ワーカーは起点を渡さないので実際には二重の歯止め。
            early_start: false,
            ..cfg.clone()
        };
        Prefetcher::spawn(build.clone(), cfg)
    });
    let mut core = PlannerCore::new(build, cfg);
    if let Some(pf) = prefetch.clone() {
        core = core.with_prefetch(pf);
    }
    Ok((core, localizer, prefetch))
}

/// compact 経路の確定出力 (`nstates × 12 B`) の置き場を決める。
///
/// - compact 以外のソルバでは常に `None` (使われない)。
/// - `compact_sink_dir` が明示されていればそれを使う。
/// - 未指定なら `compact_ram_limit_mb` に収まるかで決め、収まらなければ
///   `/tmp/vi_planner_sink` へ逃がす。小メモリ機で黙って GB 級の `RamSink` を
///   確保すると OOM killer に落とされるため。上限が 0 (既定) のときは
///   **空きメモリの半分** — 密の起動判定と同じ `MemAvailable` 基準。
///
/// 追従はパッチを置き直すたびに sink を読むので (10Hz の制御ループ)、SD カード上の
/// sink は追従の遅延に直結する。逃がす先はできるだけ速い実ディスクを
/// `compact_sink_dir` で明示すること。
fn compact_sink_dir(params: &Params, solver: U64Solver, nstates: usize) -> Option<PathBuf> {
    if !solver.caps().out_of_core {
        return None;
    }
    let bytes = nstates as u64 * 12;
    if !params.compact_sink_dir.is_empty() {
        let dir = PathBuf::from(&params.compact_sink_dir);
        eprintln!(
            "vi_planner: compact output -> disk mmap {} ({:.2} GB)",
            dir.display(),
            bytes as f64 / 1e9
        );
        return Some(dir);
    }
    // 明示された上限が最優先。0 (既定) なら空きメモリの半分を上限にする。
    let limit = match params.compact_ram_limit_mb {
        n if n > 0 => Some((n as u64) * 1024 * 1024),
        _ => mem_available().map(|avail| avail / 2),
    };
    if let Some(limit) = limit {
        if bytes > limit {
            let dir = PathBuf::from("/tmp/vi_planner_sink");
            eprintln!(
                "WARN: compact output would need {:.2} GB of RAM, over the {:.2} GB limit ({}); \
                 spilling to disk mmap {}. The follow loop re-reads the sink on every patch \
                 recenter, so point compact_sink_dir at the fastest real disk available.",
                bytes as f64 / 1e9,
                limit as f64 / 1e9,
                if params.compact_ram_limit_mb > 0 {
                    "compact_ram_limit_mb"
                } else {
                    "half of MemAvailable"
                },
                dir.display()
            );
            return Some(dir);
        }
    }
    eprintln!("vi_planner: compact output -> RAM ({:.2} GB)", bytes as f64 / 1e9);
    None
}
