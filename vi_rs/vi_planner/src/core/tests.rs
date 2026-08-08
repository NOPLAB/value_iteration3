use super::compact::{transition_reach, PenaltyOverlay};
use super::*;
use std::sync::atomic::AtomicBool;
// compact 側だけが使う型は core::compact へ移したので、テストからは直接入れる。
use vi_lib::msg::Quaternion;
use vi_lib::params::{MAX_COST, PROB_BASE_BIT};
use vi_lib::solvers::frontier2d_sparse_compact::{CompactSink, RamSink};

const RES: f64 = 0.05;

fn actions() -> Vec<Action> {
    vec![
        Action::new("forward", 0.3, 0.0, 0),
        Action::new("back", -0.2, 0.0, 1),
        Action::new("right", 0.0, -20.0, 2),
        Action::new("rightfw", 0.2, -20.0, 3),
        Action::new("left", 0.0, 20.0, 4),
        Action::new("leftfw", 0.2, 20.0, 5),
    ]
}

fn build(size: i32) -> BuildParams {
    BuildParams {
        grid: OccupancyGrid {
            width: size,
            height: size,
            resolution: RES,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            data: vec![0i8; (size * size) as usize],
        },
        actions: actions(),
        theta_cell_num: 60,
        safety_radius: 0.1,
        safety_radius_penalty: 30.0,
        goal_margin_radius: 0.2,
        goal_margin_theta: 180,
        local_xy_range: 1.0,
        patch_slack_cells: 2,
        repair_interior_cells: 16,
    }
}

fn cfg() -> PlanConfig {
    PlanConfig {
        solver: U64Solver::Frontier2DSparse,
        max_solve_iter: 100_000,
        solve_chunk: 16,
        goal_tolerance_xy: 0.25,
        goal_tolerance_deg: 10.0,
        max_rollout_steps: 10_000,
        start_tolerance_cells: 10,
        path_spacing: RES,
        action_tolerance_cells: 4,
        follow_controller: FollowKind::Greedy,
        dwa_tick_s: 0.1,
        dwa_horizon_s: 1.0,
        dwa_n_v: 7,
        dwa_n_w: 11,
        dwa_lethal_penalty: 2.0,
        mppi_samples: 256,
        mppi_lambda: 1.0,
        mppi_sigma_v: 0.0,
        mppi_sigma_w_deg: 0.0,
        compact_sink_dir: None,
        compact_sink_gen: None,
        vi_threads: 1,
        prefetch_poll_ms: 50,
        global_sweep: true,
        early_start: false,
    }
}

/// アウトオブコア経路 (states を作らない) の設定。
fn cfg_compact() -> PlanConfig {
    PlanConfig { solver: U64Solver::Frontier2DSparseCompact { band: 0 }, ..cfg() }
}

fn pose(x: f64, y: f64, yaw: f64) -> PoseView {
    PoseView { x, y, yaw_rad: yaw }
}

/// この統合の本題: 広域 (plan) と狭域 (decide) が **1 回の solve** を
/// 共有すること。旧構成 (vi_global_planner + vi_local_planner) では
/// 同じゴールに対して別プロセスで 2 回解いていた。
#[test]
fn plan_and_follow_share_a_single_solve() {
    let mut core = PlannerCore::new(build(64), cfg());
    let cancel = AtomicBool::new(false);
    let goal = pose(2.0, 2.0, 0.0);

    // 広域計画が最初の (そして唯一の) solve を走らせる。
    let (path, s1) = core.plan(pose(0.6, 0.6, 0.0), goal, &cancel).expect("plan");
    assert!(s1.solved_now && s1.iters > 0);
    assert!(path.len() > 2);

    // BT が同じゴールで FollowPath を送ってきた相当: solve は走らない。
    let s2 = core.prepare_goal(goal, &cancel).expect("prepare for follow");
    assert!(!s2.solved_now && s2.iters == 0, "follow must reuse the planner's solve");

    // その価値関数のまま追従判断が下せる。
    let robot = pose(0.6, 0.6, 0.0);
    core.set_window(robot);
    assert!(matches!(core.decide(robot), Decision::Action { .. }));

    // 1Hz リプラン相当も solve なし (ロールアウトのみ)。
    let (_, s3) = core.plan(pose(0.8, 0.9, 0.3), goal, &cancel).expect("replan");
    assert!(!s3.solved_now && s3.iters == 0);
}

/// decide → 行動適用 (並進→回転、no_noise_state_transition と同じ) を
/// 繰り返してゴール圏へ到達できること = 制御ループの中核が閉じていること。
#[test]
fn follows_policy_to_goal_on_empty_map() {
    let mut core = PlannerCore::new(build(64), cfg());
    let cancel = AtomicBool::new(false);
    let goal = pose(2.0, 2.0, 0.0);
    let stats = core.prepare_goal(goal, &cancel).expect("solve");
    assert!(stats.solved_now && stats.iters > 0);

    let (mut x, mut y, mut yaw) = (0.6f64, 0.6f64, 0.0f64);
    for _ in 0..500 {
        let p = pose(x, y, yaw);
        core.set_window(p);
        core.refine_passes(1);
        match core.decide(p) {
            Decision::Goal => {
                let d = core.goal_distance(x, y).unwrap();
                assert!(d <= 0.3, "goal margin: d = {d}");
                return;
            }
            Decision::Action { fw, rot_deg, .. } => {
                x += fw * yaw.cos();
                y += fw * yaw.sin();
                yaw += rot_deg.to_radians();
            }
            Decision::NoAction => panic!("no action at ({x:.2}, {y:.2})"),
        }
    }
    panic!("did not reach the goal in 500 steps");
}

/// **compact 経路の本題**: `states` を確保せずに解いた場でも、追従ループ
/// (set_window → refine → decide) がゴール圏まで走り切れること。パッチは
/// 走行中に何度も置き直される (`needs_recenter`)。
#[test]
fn compact_follows_policy_to_goal() {
    let mut core = PlannerCore::new(build(96), cfg_compact());
    let cancel = AtomicBool::new(false);
    let goal = pose(4.0, 4.0, 0.0);
    let stats = core.prepare_goal(goal, &cancel).expect("compact solve");
    assert!(stats.solved_now && stats.iters > 0);

    let (mut x, mut y, mut yaw) = (0.6f64, 0.6f64, 0.0f64);
    let mut hydrations = 0usize;
    let mut last_at = None;
    for _ in 0..500 {
        let p = pose(x, y, yaw);
        core.set_window(p);
        let at = core.patch.as_ref().and_then(|p| p.at);
        if at != last_at {
            hydrations += 1;
            last_at = at;
        }
        core.refine_passes(1);
        match core.decide(p) {
            Decision::Goal => {
                let d = core.goal_distance(x, y).unwrap();
                assert!(d <= 0.3, "goal margin: d = {d}");
                assert!(hydrations > 1, "the patch must have moved along the way");
                return;
            }
            Decision::Action { fw, rot_deg, .. } => {
                x += fw * yaw.cos();
                y += fw * yaw.sin();
                yaw += rot_deg.to_radians();
            }
            Decision::NoAction => panic!("no action at ({x:.2}, {y:.2})"),
        }
    }
    panic!("did not reach the goal in 500 steps");
}

/// 上のテストは 0.05m セルなのでパッチ (99 セル角) が地図より大きく、凍結境界は
/// ほぼ地図外の壁になる。こちらは **map_tsudanuma と同じ幾何** (0.25m セル、
/// 歩幅 0.5m → パッチ 27 セル角) で、パッチが地図の内側に完全に収まったまま
/// 何度も置き直される経路を走らせる。凍結境界の 4 辺すべてに compact の実値が
/// 入る唯一のケースで、1 セルずれると「1.5m 走るごとに、ある方位でだけ NoAction」
/// という気付きにくい壊れ方をする。
#[test]
fn compact_recenters_repeatedly_with_an_interior_patch() {
    // 120x120 @0.25m = 30m x 30m。
    let mut b = build(120);
    b.grid.resolution = 0.25;
    b.actions = vec![
        Action::new("forward", 0.5, 0.0, 0),
        Action::new("back", -0.3333, 0.0, 1),
        Action::new("right", 0.0, -20.0, 2),
        Action::new("rightfw", 0.3333, -20.0, 3),
        Action::new("left", 0.0, 20.0, 4),
        Action::new("leftfw", 0.3333, 20.0, 5),
    ];
    b.goal_margin_radius = 0.5; // セルサイズに比例 (overrides と同じ)
    b.safety_radius_penalty = 1.0;

    let mut core = PlannerCore::new(b, cfg_compact());
    let cancel = AtomicBool::new(false);
    let goal = pose(25.0, 25.0, 0.0);
    core.prepare_goal(goal, &cancel).expect("compact solve");

    let (mut x, mut y, mut yaw) = (4.0f64, 4.0f64, 0.0f64);
    let (mut hydrations, mut interior) = (0usize, 0usize);
    let mut last_at = None;
    for _ in 0..500 {
        let p = pose(x, y, yaw);
        core.set_window(p);
        let patch = core.patch.as_ref().unwrap();
        if patch.at != last_at {
            hydrations += 1;
            last_at = patch.at;
            let (p0x, p0y) = patch.at.unwrap();
            let side_max = 2 * patch.half;
            if p0x >= 0 && p0y >= 0 && p0x + side_max < 120 && p0y + side_max < 120 {
                interior += 1;
            }
        }
        core.refine_passes(1);
        match core.decide(p) {
            Decision::Goal => {
                assert!(hydrations >= 3, "patch must move several times: {hydrations}");
                assert!(
                    interior >= 3,
                    "the patch must sit strictly inside the map at least a few times, \
                     so all four frozen edges carry real compact values: {interior}"
                );
                return;
            }
            Decision::Action { fw, rot_deg, .. } => {
                x += fw * yaw.cos();
                y += fw * yaw.sin();
                yaw += rot_deg.to_radians();
            }
            Decision::NoAction => panic!(
                "no action at ({x:.2}, {y:.2}) yaw={:.0}deg after {hydrations} hydrations",
                yaw.to_degrees()
            ),
        }
    }
    panic!("did not reach the goal in 500 steps ({hydrations} hydrations)");
}

/// DWA (連続行動) コントローラでも同じ場でゴール圏まで走り切れること。greedy と
/// 違い指令は速度なので、実機と同じく tick (0.1 s) ごとに定速弧で積分して再決定
/// する (greedy 系テストの「fw を 1 歩ぶん跳ぶ」とは実行モデルが違う)。
#[test]
fn dwa_follows_to_goal_on_empty_map() {
    use vi_lib::ctrl::unicycle_step;
    let mut c = cfg();
    c.follow_controller = FollowKind::Dwa;
    let mut core = PlannerCore::new(build(64), c);
    let cancel = AtomicBool::new(false);
    let goal = pose(2.0, 2.0, 0.0);
    core.prepare_goal(goal, &cancel).expect("solve");

    let tick = 0.1f64;
    let (mut x, mut y, mut yaw) = (0.6f64, 0.6f64, 0.0f64);
    let mut saw_dwa = false;
    for _ in 0..3000 {
        let p = pose(x, y, yaw);
        core.set_window(p);
        match core.decide(p) {
            Decision::Goal => {
                let d = core.goal_distance(x, y).unwrap();
                assert!(d <= 0.3, "goal margin: d = {d}");
                assert!(saw_dwa, "DWA must have decided at least once (not only fallback)");
                return;
            }
            Decision::Action { id, fw, rot_deg } => {
                // DWA 自身の指令は id を持たない (greedy 救済のときだけ Some)。
                saw_dwa |= id.is_none();
                let (nx, ny, nyaw) = unicycle_step(x, y, yaw, fw, rot_deg.to_radians(), tick);
                x = nx;
                y = ny;
                yaw = nyaw;
            }
            Decision::NoAction => panic!("no action at ({x:.2}, {y:.2})"),
        }
    }
    panic!("did not reach the goal in 3000 ticks");
}

/// compact 経路 (パッチ上) でも DWA が走り切れること。ホライズンの端がパッチ外に
/// 出る候補は V̂ 評価不能で棄却されるだけ (凍結境界の不変条件はそのまま) で、
/// パッチは走行中に置き直され続ける。
#[test]
fn dwa_follows_to_goal_on_compact_patch() {
    use vi_lib::ctrl::unicycle_step;
    let mut c = cfg_compact();
    c.follow_controller = FollowKind::Dwa;
    let mut core = PlannerCore::new(build(96), c);
    let cancel = AtomicBool::new(false);
    let goal = pose(4.0, 4.0, 0.0);
    core.prepare_goal(goal, &cancel).expect("compact solve");

    let tick = 0.1f64;
    let (mut x, mut y, mut yaw) = (0.6f64, 0.6f64, 0.0f64);
    let (mut saw_dwa, mut hydrations) = (false, 0usize);
    let mut last_at = None;
    for _ in 0..6000 {
        let p = pose(x, y, yaw);
        core.set_window(p);
        let at = core.patch.as_ref().and_then(|p| p.at);
        if at != last_at {
            hydrations += 1;
            last_at = at;
        }
        // refine はしない (スキャンを入れないので値は動かない)。decide だけを回す。
        match core.decide(p) {
            Decision::Goal => {
                let d = core.goal_distance(x, y).unwrap();
                assert!(d <= 0.3, "goal margin: d = {d}");
                assert!(saw_dwa, "DWA must have decided at least once (not only fallback)");
                assert!(hydrations > 1, "the patch must have moved along the way");
                return;
            }
            Decision::Action { id, fw, rot_deg } => {
                saw_dwa |= id.is_none();
                let (nx, ny, nyaw) = unicycle_step(x, y, yaw, fw, rot_deg.to_radians(), tick);
                x = nx;
                y = ny;
                yaw = nyaw;
            }
            Decision::NoAction => panic!("no action at ({x:.2}, {y:.2})"),
        }
    }
    panic!("did not reach the goal in 6000 ticks");
}

/// DWA でも方策なしセル (膨張内・非 free) からは greedy の近傍救済が効くこと —
/// 候補全滅時のフォールバックが `decide_borrows_action_from_neighbors` と同じ
/// 意味論を保つ。
#[test]
fn dwa_falls_back_to_greedy_on_unevaluable_cells() {
    let size = 64;
    let mut b = build(size);
    for y in 18..=22 {
        for x in 18..=22 {
            b.grid.data[(y * size + x) as usize] = 100;
        }
    }
    let mut c = cfg();
    c.follow_controller = FollowKind::Dwa;
    let mut core = PlannerCore::new(b, c);
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.5, 2.5, 0.0), &cancel).expect("solve");

    // 障害物セルの上: DWA は評価不能 → greedy 借用が離散行動 (id: Some) を返す。
    let on_obstacle = pose(20.5 * RES, 20.5 * RES, 0.0);
    assert!(matches!(core.decide(on_obstacle), Decision::Action { id: Some(_), .. }));
}

/// MPPI (連続行動) でも同じ場でゴール圏まで走り切れること。tick 間状態 (warm
/// start の名目列) は MppiController が Mutex で持ち、decide は &self のまま。
#[test]
fn mppi_follows_to_goal_on_empty_map() {
    use vi_lib::ctrl::unicycle_step;
    let mut c = cfg();
    c.follow_controller = FollowKind::Mppi;
    let mut core = PlannerCore::new(build(64), c);
    let cancel = AtomicBool::new(false);
    let goal = pose(2.0, 2.0, 0.0);
    core.prepare_goal(goal, &cancel).expect("solve");

    let tick = 0.1f64;
    let (mut x, mut y, mut yaw) = (0.6f64, 0.6f64, 0.0f64);
    let mut saw_mppi = false;
    for _ in 0..3000 {
        let p = pose(x, y, yaw);
        core.set_window(p);
        match core.decide(p) {
            Decision::Goal => {
                let d = core.goal_distance(x, y).unwrap();
                assert!(d <= 0.3, "goal margin: d = {d}");
                assert!(saw_mppi, "MPPI must have decided at least once (not only fallback)");
                return;
            }
            Decision::Action { id, fw, rot_deg } => {
                saw_mppi |= id.is_none();
                let (nx, ny, nyaw) = unicycle_step(x, y, yaw, fw, rot_deg.to_radians(), tick);
                x = nx;
                y = ny;
                yaw = nyaw;
            }
            Decision::NoAction => panic!("no action at ({x:.2}, {y:.2})"),
        }
    }
    panic!("did not reach the goal in 3000 ticks");
}

/// MPPI でも方策なしセルからは greedy の近傍救済が効くこと (フォールバック時は
/// 名目列を捨てる経路も通る)。
#[test]
fn mppi_falls_back_to_greedy_on_unevaluable_cells() {
    let size = 64;
    let mut b = build(size);
    for y in 18..=22 {
        for x in 18..=22 {
            b.grid.data[(y * size + x) as usize] = 100;
        }
    }
    let mut c = cfg();
    c.follow_controller = FollowKind::Mppi;
    let mut core = PlannerCore::new(b, c);
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.5, 2.5, 0.0), &cancel).expect("solve");

    let on_obstacle = pose(20.5 * RES, 20.5 * RES, 0.0);
    assert!(matches!(core.decide(on_obstacle), Decision::Action { id: Some(_), .. }));
}

/// compact 経路の広域側が密経路と同じ経路を返すこと (ロールアウトは sink を読む)。
#[test]
fn compact_plan_matches_dense() {
    let cancel = AtomicBool::new(false);
    let goal = pose(2.0, 2.0, 0.0);
    let start = pose(0.6, 0.6, 0.0);

    let (dense, _) =
        PlannerCore::new(build(64), cfg()).plan(start, goal, &cancel).expect("dense plan");

    let mut core = PlannerCore::new(build(64), cfg_compact());
    let (p1, s1) = core.plan(start, goal, &cancel).expect("compact plan");
    assert!(s1.solved_now && s1.iters > 0);
    assert_eq!(p1.len(), dense.len(), "compact path length differs from dense");
    for (a, b) in p1.iter().zip(dense.iter()) {
        assert!(
            (a.x - b.x).abs() < 1e-12 && (a.y - b.y).abs() < 1e-12,
            "compact pose {a:?} != dense pose {b:?}"
        );
    }
    // 同一ゴールの再計画は solve なし (キャッシュヒット)。
    let (_, s2) = core.plan(pose(0.4, 1.8, 1.0), goal, &cancel).expect("compact replan");
    assert!(!s2.solved_now);
}

/// パッチのハイドレートが compact の場を忠実に写していること。
/// ウィンドウ内の各セルについて、パッチの `(total_cost, optimal_action, free)` が
/// sink / 静的地図と一致する = 凍結境界とローカル反復の前提。
#[test]
fn hydrated_patch_matches_the_compact_field() {
    let mut core = PlannerCore::new(build(64), cfg_compact());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("compact solve");
    let robot = pose(1.0, 1.0, 0.0);
    core.set_window(robot);

    let p = core.patch.as_ref().expect("patch built");
    let (p0x, p0y) = p.at.expect("patch hydrated");
    // 遷移がパッチからはみ出さないこと (凍結境界の成立条件)。
    assert!(p.vi.local_ixy_range + p.reach < p.half);

    let Some(CachedGoal { field: Field::Compact(f), .. }) = core.cached.as_ref() else {
        panic!("compact field expected");
    };
    let nt = f.cell_num.2;
    let mut checked = 0usize;
    for py in 0..=(2 * p.half) {
        for px in 0..=(2 * p.half) {
            let (gx, gy) = (p0x + px, p0y + py);
            if gx < 0 || gy < 0 || gx >= f.cell_num.0 || gy >= f.cell_num.1 {
                continue;
            }
            for it in [0, nt / 3, nt - 1] {
                let (v, a) = f.sink.read(f.orig(gx, gy, it));
                let s = &p.vi.base.states[p.vi.base.to_index(px, py, it) as usize];
                assert_eq!(s.total_cost, v, "value at ({gx},{gy},{it})");
                assert_eq!(
                    s.optimal_action,
                    if a >= 0 { Some(a as usize) } else { None },
                    "policy at ({gx},{gy},{it})"
                );
                assert_eq!(s.final_state, v == 0, "final_state at ({gx},{gy},{it})");
                assert!(s.free, "empty map: every in-map cell is free");
                checked += 1;
            }
        }
    }
    assert!(checked > 100, "the patch must overlap the map");
}

/// ディスク mmap sink 経由でも追従判断が出ること (Pi4 のような小メモリ機の経路)。
#[test]
fn compact_with_mmap_sink_follows() {
    let dir = std::env::temp_dir().join("vi_planner_core_mmap_test");
    let cfg = PlanConfig { compact_sink_dir: Some(dir.clone()), ..cfg_compact() };
    let mut core = PlannerCore::new(build(64), cfg);
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("solve");
    let robot = pose(0.6, 0.6, 0.0);
    core.set_window(robot);
    assert!(matches!(core.decide(robot), Decision::Action { .. }));
    drop(core);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn plans_and_caches_for_same_goal() {
    let mut core = PlannerCore::new(build(64), cfg());
    let cancel = AtomicBool::new(false);
    let goal = pose(2.0, 2.0, 0.0);

    let (p1, s1) = core.plan(pose(0.6, 0.6, 0.0), goal, &cancel).expect("first plan");
    assert!(s1.solved_now && s1.iters > 0);
    assert!(p1.len() > 2);

    let (p2, s2) = core.plan(pose(0.4, 1.8, 1.0), goal, &cancel).expect("replan");
    assert!(!s2.solved_now && s2.iters == 0);
    assert!(p2.len() > 2);

    let (_, s3) =
        core.plan(pose(0.6, 0.6, 0.0), pose(0.8, 2.4, 0.0), &cancel).expect("new goal");
    assert!(s3.solved_now);
}

/// `is_cached_goal` は追従スレッドの世代チェックの土台。別ゴールの計画で
/// キャッシュが差し替わったら false になること。
#[test]
fn is_cached_goal_tracks_the_replacement() {
    let mut core = PlannerCore::new(build(64), cfg());
    let cancel = AtomicBool::new(false);
    let followed = pose(2.0, 2.0, 0.0);

    assert!(!core.is_cached_goal(followed), "nothing solved yet");
    core.prepare_goal(followed, &cancel).expect("solve");
    assert!(core.is_cached_goal(followed));
    // 許容差内のゆらぎは同一ゴール扱い。
    assert!(core.is_cached_goal(pose(2.05, 2.0, 0.0)));

    // 広域側が別ゴールを解くとキャッシュが差し替わる。
    core.plan(pose(0.6, 0.6, 0.0), pose(0.8, 2.4, 0.0), &cancel).expect("new goal");
    assert!(!core.is_cached_goal(followed));
}

/// ゴールが変わったらパッチは無効化され、次の `set_window` で新しい場から
/// 起こし直されること (古いゴールの方策で走らせない)。
#[test]
fn compact_patch_is_invalidated_on_a_new_goal() {
    let mut core = PlannerCore::new(build(64), cfg_compact());
    let cancel = AtomicBool::new(false);
    let robot = pose(0.6, 0.6, 0.0);

    core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("solve");
    core.set_window(robot);
    assert!(core.patch.as_ref().unwrap().at.is_some());

    core.prepare_goal(pose(0.8, 2.4, 0.0), &cancel).expect("new goal");
    assert!(core.patch.as_ref().unwrap().at.is_none(), "stale patch must be dropped");
    assert_eq!(core.decide(robot), Decision::NoAction, "no policy before set_window");
    core.set_window(robot);
    assert!(matches!(core.decide(robot), Decision::Action { .. }));
}

#[test]
fn densified_path_spacing_bounded() {
    let mut core = PlannerCore::new(build(64), cfg());
    let cancel = AtomicBool::new(false);
    let (p, _) = core.plan(pose(0.6, 0.6, 0.0), pose(2.0, 2.0, 0.0), &cancel).unwrap();
    for w in p.windows(2) {
        let d = ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt();
        assert!(d <= RES + 1e-9);
    }
}

#[test]
fn pre_raised_cancel_aborts_solve() {
    let mut core = PlannerCore::new(build(64), cfg());
    let cancel = AtomicBool::new(true);
    assert_eq!(
        core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).unwrap_err(),
        PlanError::Cancelled
    );
    let mut core = PlannerCore::new(build(64), cfg());
    assert_eq!(
        core.plan(pose(0.6, 0.6, 0.0), pose(2.0, 2.0, 0.0), &cancel).unwrap_err(),
        PlanError::Cancelled
    );
    // compact 経路も同じ (中断はソルバ内部のラウンド境界で観測される)。
    let mut core = PlannerCore::new(build(64), cfg_compact());
    assert_eq!(
        core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).unwrap_err(),
        PlanError::Cancelled
    );
}

/// 進捗コールバックは新規 solve でのみ発火し、キャッシュヒットでは発火しない
/// (途中経過の value_function 配信の前提)。
#[test]
fn progress_callback_fires_only_when_solving() {
    let mut core = PlannerCore::new(build(64), cfg());
    let cancel = AtomicBool::new(false);
    let goal = pose(2.0, 2.0, 0.0);

    let mut calls = 0usize;
    core.plan_with_progress(pose(0.6, 0.6, 0.0), goal, &cancel, &mut |vi| {
        calls += 1;
        assert!(vi.cell_num().0 > 0);
    })
    .expect("first plan");
    assert!(calls > 0);

    let mut calls_cached = 0usize;
    core.prepare_goal_with_progress(goal, None, &cancel, &mut |_| calls_cached += 1)
        .expect("follow");
    assert_eq!(calls_cached, 0);
}

/// value_grid は全域、window_value_grid はクランプ後の実ウィンドウと
/// 寸法・原点・データ長が一致し、値は OccupancyGrid の規約 (-1..=100) に収まる。
#[test]
fn visualization_grids_match_geometry() {
    let mut core = PlannerCore::new(build(64), cfg());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("solve");

    let g = core.value_grid(60).expect("full grid");
    assert_eq!((g.width, g.height), (64, 64));
    assert_eq!(g.data.len(), 64 * 64);

    // 地図端 (原点) にウィンドウ → min 側がクランプされ 21x21
    // (local_ixy_range = 1.0m / 0.05m = 20)。
    let robot = pose(0.0, 0.0, 0.0);
    core.set_window(robot);
    let w = core.window_value_grid(robot, 60).expect("window grid");
    assert_eq!((w.width, w.height), (21, 21));
    assert_eq!(w.data.len(), (w.width * w.height) as usize);
    assert_eq!((w.origin_x, w.origin_y), (0.0, 0.0));
    assert!(w.data.iter().all(|&v| (-1..=100).contains(&i32::from(v))));

    let empty = PlannerCore::new(build(64), cfg());
    assert!(empty.value_grid(60).is_none());
    assert!(empty.window_value_grid(robot, 60).is_none());
}

/// compact 経路でも可視化グリッドが同じ幾何で出ること (全域は sink 走査)。
#[test]
fn compact_visualization_grids_match_geometry() {
    let mut core = PlannerCore::new(build(64), cfg_compact());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("solve");

    let g = core.value_grid(60).expect("full grid");
    assert_eq!((g.width, g.height), (64, 64));
    assert_eq!(g.data.len(), 64 * 64);

    let robot = pose(1.0, 1.0, 0.0);
    core.set_window(robot);
    let w = core.window_value_grid(robot, 60).expect("window grid");
    // ウィンドウはパッチの内側に丸ごと収まるのでクランプされない
    // (local_ixy_range = 1.0m / 0.05m = 20 → 41x41)。密経路で地図端に寄せたとき
    // だけ 21x21 にクランプされる (visualization_grids_match_geometry 参照)。
    assert_eq!((w.width, w.height), (41, 41));
    assert!(w.data.iter().all(|&v| (-1..=100).contains(&i32::from(v))));
}

/// スキャンで注入された local_penalty が、ローカル反復を経て「ヒット帯へ
/// 踏み込む行動を持つ上流セル」の価値を引き上げること (障害物回避の根拠)。
#[test]
fn scan_penalty_raises_upstream_value() {
    let mut core = PlannerCore::new(build(64), cfg());
    let cancel = AtomicBool::new(false);
    // ゴールは東 (x 正方向) の先。前進が最短。
    core.prepare_goal(pose(2.5, 1.0, 0.0), &cancel).expect("solve");

    let robot = pose(1.0, 1.0, 0.0);
    core.set_window(robot);

    // 前進 1 ステップ (0.3m) でヒット帯 (1.5, 1.0)±2 セルに着地する上流セル。
    let (uix, uiy, uit) = {
        let vi = core.local().unwrap();
        pose_to_cell(&vi.base, 1.2, 1.0, 0.0)
    };
    let before = {
        let vi = core.local().unwrap();
        vi.base.states[vi.base.to_index(uix, uiy, uit) as usize].total_cost
    };

    // 正面 0.5m にヒット 1 ビーム → (1.5, 1.0) 周辺へ 2048<<PROB_BASE_BIT。
    let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.5] };
    core.observe_scan(&scan, robot);
    {
        let vi = core.local().unwrap();
        let (hx, hy, _) = pose_to_cell(&vi.base, 1.5, 1.0, 0.0);
        let hit = vi.base.to_index(hx, hy, 0) as usize;
        assert_eq!(vi.base.states[hit].local_penalty, 2048u64 << PROB_BASE_BIT);
    }

    core.refine_passes(5);
    let after = {
        let vi = core.local().unwrap();
        vi.base.states[vi.base.to_index(uix, uiy, uit) as usize].total_cost
    };
    assert!(after > before, "upstream value must rise: before={before}, after={after}");
}

/// 同じことが compact 経路のパッチ上でも成り立つこと (狭域が compact の場の
/// 上でも障害物回避として機能する = この実装の目的)。
#[test]
fn compact_scan_penalty_raises_upstream_value() {
    let mut core = PlannerCore::new(build(64), cfg_compact());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.5, 1.0, 0.0), &cancel).expect("solve");

    let robot = pose(1.0, 1.0, 0.0);
    core.set_window(robot);

    let (uix, uiy, uit) = {
        let vi = core.local().unwrap();
        pose_to_cell(&vi.base, 1.2, 1.0, 0.0)
    };
    let before = {
        let vi = core.local().unwrap();
        vi.base.states[vi.base.to_index(uix, uiy, uit) as usize].total_cost
    };
    assert!(before < MAX_COST, "the patch must carry the compact solution");

    let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.5] };
    core.observe_scan(&scan, robot);
    core.refine_passes(5);
    let after = {
        let vi = core.local().unwrap();
        vi.base.states[vi.base.to_index(uix, uiy, uit) as usize].total_cost
    };
    assert!(after > before, "upstream value must rise: before={before}, after={after}");
}

/// 品質ゲート: quality → 減衰段数の量子化。gate 以上・ゲート無効・External
/// (quality 1.0) は 0、quality が半分ごとに 1 段、床は 11 (注入 1、0 にしない)。
#[test]
fn quality_shift_quantizes_by_halving() {
    assert_eq!(quality_shift(1.0, 0.25), 0); // External / 良好フィット
    assert_eq!(quality_shift(0.25, 0.25), 0); // gate ちょうど
    assert_eq!(quality_shift(0.5, 0.0), 0); // ゲート無効
    assert_eq!(quality_shift(0.125, 0.25), 1); // gate の 1/2 → 2048>>1
    assert_eq!(quality_shift(0.0625, 0.25), 2); // gate の 1/4 → 2048>>2
    assert_eq!(quality_shift(0.0, 0.25), 11); // 完全ミスマッチでも床 11
    assert_eq!(quality_shift(1e-12, 0.25), 11); // クランプ
}

/// 品質ゲート付き注入がヒット帯へ減衰値を書くこと (shift 0 は従来と同一)。
#[test]
fn observe_scan_gated_attenuates_injection() {
    let mut core = PlannerCore::new(build(64), cfg());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.5, 1.0, 0.0), &cancel).expect("solve");
    let robot = pose(1.0, 1.0, 0.0);
    core.set_window(robot);

    let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.5] };
    core.observe_scan_gated(&scan, robot, 3);
    let vi = core.local().unwrap();
    let (hx, hy, _) = pose_to_cell(&vi.base, 1.5, 1.0, 0.0);
    let hit = vi.base.to_index(hx, hy, 0) as usize;
    assert_eq!(vi.base.states[hit].local_penalty, 256u64 << PROB_BASE_BIT);
}

/// penalty 表が `set_local_cost` の書く値を**丸めずに**持てること
/// (1 B/セルで足りる根拠 = 2 の冪しか来ない)。
#[test]
fn penalty_overlay_is_exact_for_every_value_set_local_cost_writes() {
    let mut ov = PenaltyOverlay::new(4, 3);
    assert_eq!(ov.get(0, 0), 0, "初期値は penalty 無し");

    // set_local_cost が書くのは 2048<<PROB_BASE_BIT と、その半減列だけ。
    let mut v = 2048u64 << PROB_BASE_BIT;
    let mut seen = 0;
    while v > 0 {
        ov.set(1, 2, v);
        assert_eq!(ov.get(1, 2), v, "{v} を丸めずに持てること");
        v /= 2;
        seen += 1;
    }
    assert_eq!(seen, 30, "2^29 から 1 まで 30 段");

    ov.set(1, 2, 0);
    assert_eq!(ov.get(1, 2), 0);
    // 表の外は 0 (パッチは地図外へ食い込む)。
    ov.set(-1, 0, 1 << 20);
    ov.set(4, 0, 1 << 20);
    assert_eq!((ov.get(-1, 0), ov.get(4, 0), ov.get(0, 3)), (0, 0, 0));

    ov.set(0, 0, 1 << 20);
    ov.clear();
    assert_eq!(ov.get(0, 0), 0, "ゴールを取り直したら消えること");
}

/// compact 経路の狭域の成果が **sink に載る** こと = 広域と場を共有すること。
/// 密経路の `states` に相当する共有場が compact でも成立している、の実体。
#[test]
fn compact_local_refinement_reaches_the_shared_sink() {
    let mut core = PlannerCore::new(build(64), cfg_compact());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.5, 1.0, 0.0), &cancel).expect("solve");

    let robot = pose(1.0, 1.0, 0.0);
    core.set_window(robot);
    // 前進 1 ステップでヒット帯へ着地する上流セル (dense 版と同じ取り方)。
    let (uix, uiy, uit) = {
        let vi = core.local().unwrap();
        pose_to_cell(&vi.base, 1.2, 1.0, 0.0)
    };
    let (p0x, p0y) = core.patch.as_ref().unwrap().at.unwrap();
    let (gx, gy) = (p0x + uix, p0y + uiy);
    let sink_of = |core: &PlannerCore| {
        let Some(CachedGoal { field: Field::Compact(f), .. }) = core.cached.as_ref() else {
            panic!("compact field expected");
        };
        f.sink.read(f.orig(gx, gy, uit))
    };
    let before = sink_of(&core);

    let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.5] };
    core.observe_scan(&scan, robot);
    core.refine_passes(5);

    let after = sink_of(&core);
    assert!(
        after.0 > before.0,
        "狭域が上げた値が sink に返っていること: before={before:?}, after={after:?}"
    );
}

/// 書き戻したあとも広域が経路を返せること。
///
/// sink は `total_cost >= MAX_COST` を「未到達」として扱う
/// (`CompactPolicy::action_index`) ので、書き戻しでそこへ踏み込むと、これまで
/// 通っていた `compute_path_to_pose` が黙って失敗に変わる。penalty は加算で
/// あって通行止めではない (`action_cost_raw` が MAX_COST を返すのは遷移先が
/// 非 free のときだけ) ので起きないはずだが、退行するならここなので張っておく。
#[test]
fn compact_plan_still_succeeds_after_the_window_is_committed() {
    let mut core = PlannerCore::new(build(64), cfg_compact());
    let cancel = AtomicBool::new(false);
    let goal = pose(2.5, 1.0, 0.0);
    let start = pose(0.4, 1.0, 0.0);
    let (before, _) = core.plan(start, goal, &cancel).expect("plan before");

    let robot = pose(1.0, 1.0, 0.0);
    core.set_window(robot);
    let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.5] };
    core.observe_scan(&scan, robot);
    core.refine_passes(5);

    let (after, s) = core.plan(start, goal, &cancel).expect("plan after commit");
    assert!(!s.solved_now, "同じゴールなので解き直さないこと");
    assert!(after.len() > 2, "経路が返ること (未到達扱いになっていない)");
    assert_ne!(
        before.len(),
        after.len(),
        "狭域が上げた値が広域のロールアウトに効いていること (経路が変わる)"
    );
}

/// この変更の主張そのもの: penalty を入れたあとの compact が、`global_sweep` を
/// 切った密経路と**同じ**振る舞いになること。
///
/// 上流セルの値が両者で一致するところまで見る (compact 側はパッチ経由・sink
/// 経由の 2 段を挟むので、揃わなければどこかで場が食い違っている)。
#[test]
fn compact_with_penalties_matches_dense_without_a_global_sweep() {
    let goal = pose(2.5, 1.0, 0.0);
    let robot = pose(1.0, 1.0, 0.0);
    let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.5] };
    let cancel = AtomicBool::new(false);

    let mut dense = PlannerCore::new(build(64), cfg());
    dense.prepare_goal(goal, &cancel).expect("dense solve");
    dense.set_window(robot);
    // 前進 1 ステップでヒット帯へ着地する上流セル (密経路のセル座標 = 全域座標)。
    let (gix, giy, git) = {
        let vi = dense.local().unwrap();
        pose_to_cell(&vi.base, 1.2, 1.0, 0.0)
    };
    dense.observe_scan(&scan, robot);
    dense.refine_passes(5);
    let dense_value = {
        let vi = dense.local().unwrap();
        vi.base.states[vi.base.to_index(gix, giy, git) as usize].total_cost
    };

    let mut compact = PlannerCore::new(build(64), cfg_compact());
    compact.prepare_goal(goal, &cancel).expect("compact solve");
    compact.set_window(robot);
    compact.observe_scan(&scan, robot);
    compact.refine_passes(5);
    let compact_value = {
        let Some(CachedGoal { field: Field::Compact(f), .. }) = compact.cached.as_ref() else {
            panic!("compact field expected");
        };
        f.sink.read(f.orig(gix, giy, git)).0
    };

    assert_eq!(
        compact_value, dense_value,
        "compact の sink が密の states と同じ値を持つこと (上流セル {gix},{giy},{git})"
    );
}

/// パッチを置き直しても、狭域が上げた値が戻らないこと。
///
/// 値は sink から復元でき、それを裏付ける `local_penalty` は penalty 表から
/// 復元できる。**表が無いと**、起こし直した直後の価値反復が penalty 0 で回って
/// 値を静かに元へ戻す (だからこの 2 つはセット)。
#[test]
fn compact_penalty_survives_a_patch_recenter() {
    let mut core = PlannerCore::new(build(64), cfg_compact());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.5, 1.0, 0.0), &cancel).expect("solve");

    let robot = pose(1.0, 1.0, 0.0);
    core.set_window(robot);
    let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.5] };
    core.observe_scan(&scan, robot);
    core.refine_passes(5);

    // ヒット帯 (1.5, 1.0) のグローバルセルと、そこに載った penalty。
    let (hgx, hgy) = {
        let p = core.patch.as_ref().unwrap();
        let (p0x, p0y) = p.at.unwrap();
        let (hx, hy, _) = pose_to_cell(&p.vi.base, 1.5, 1.0, 0.0);
        (p0x + hx, p0y + hy)
    };
    assert_eq!(
        core.penalty.as_ref().unwrap().get(hgx, hgy),
        2048u64 << PROB_BASE_BIT,
        "ヒット帯の penalty が表に載ること"
    );
    let raised = {
        let Some(CachedGoal { field: Field::Compact(f), .. }) = core.cached.as_ref() else {
            panic!("compact field expected");
        };
        f.sink.read(f.orig(hgx, hgy, 0)).0
    };

    // 遠くへ動かしてパッチを置き直させ、戻ってくる。
    let away = pose(1.0, 2.6, 0.0);
    core.set_window(away);
    let moved = core.patch.as_ref().unwrap().at;
    core.set_window(robot);
    assert_ne!(moved, core.patch.as_ref().unwrap().at, "パッチが置き直されていること");

    let p = core.patch.as_ref().unwrap();
    let (p0x, p0y) = p.at.unwrap();
    let idx = p.vi.base.to_index(hgx - p0x, hgy - p0y, 0) as usize;
    let s = &p.vi.base.states[idx];
    assert_eq!(s.local_penalty, 2048u64 << PROB_BASE_BIT, "penalty が復元されること");
    assert_eq!(s.total_cost, raised, "sink から上がった値のまま起きること");

    // 起こし直した場でもう一度回しても値は戻らない (penalty が効いているため)。
    core.refine_passes(3);
    let p = core.patch.as_ref().unwrap();
    let after = &p.vi.base.states[p.vi.base.to_index(hgx - p0x, hgy - p0y, 0) as usize];
    assert!(after.total_cost >= raised, "penalty 抜きで解き直されていないこと");
}

/// 狭域が何もしていない tick では sink を書かないこと (mmap sink のページを
/// 無駄に汚さない)。
#[test]
fn compact_commit_writes_nothing_when_the_window_is_settled() {
    /// 書き込み回数を数える sink。
    struct CountingSink {
        inner: RamSink,
        writes: std::sync::Arc<std::sync::atomic::AtomicUsize>,
    }
    impl CompactSink for CountingSink {
        fn write_column(&mut self, base: usize, values: &[u64], actions: &[i32]) {
            self.writes.fetch_add(1, Ordering::Relaxed);
            self.inner.write_column(base, values, actions);
        }
        fn read(&self, orig: usize) -> (u64, i32) {
            self.inner.read(orig)
        }
    }

    let mut core = PlannerCore::new(build(64), cfg_compact());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("solve");

    // solve 済みの sink を数える sink で包み直す。
    let writes = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    {
        let Some(CachedGoal { field: Field::Compact(f), .. }) = core.cached.as_mut() else {
            panic!("compact field expected");
        };
        let (nx, ny, nt) = f.cell_num;
        let n = nx as usize * ny as usize * nt as usize;
        let mut inner = RamSink::new(n);
        for orig in 0..n {
            let (v, a) = f.sink.read(orig);
            inner.write_column(orig, &[v], &[a]);
        }
        f.sink = Box::new(CountingSink { inner, writes: writes.clone() });
    }

    let robot = pose(1.0, 1.0, 0.0);
    core.set_window(robot);
    core.refine_passes(2); // 均す
    writes.store(0, Ordering::Relaxed);
    core.refine_passes(2); // 何も動かないはず
    assert_eq!(writes.load(Ordering::Relaxed), 0, "収束済みの窓では書かないこと");

    let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.5] };
    core.observe_scan(&scan, robot);
    core.refine_passes(2);
    assert!(writes.load(Ordering::Relaxed) > 0, "動いたぶんは書くこと");
}

/// 障害物・ペナルティ変化の無いウィンドウは 1 パスで Δ=0 になり、
/// refine_for が予算を使い切らず早期リターンすること。
#[test]
fn refine_early_exits_when_window_converged() {
    let mut core = PlannerCore::new(build(64), cfg());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("solve");
    core.set_window(pose(1.0, 1.0, 0.0));
    core.refine_passes(2); // 念のため均しておく
    let t0 = Instant::now();
    let delta = core.refine_for(Duration::from_secs(10));
    assert_eq!(delta, 0);
    assert!(t0.elapsed() < Duration::from_secs(5), "must not burn the whole budget");
}

/// 方策なしセル (障害物の膨張内) からの近傍救済。
#[test]
fn decide_borrows_action_from_neighbors() {
    let size = 64;
    let mut b = build(size);
    // (20, 20) 付近に小さな障害物ブロック。
    for y in 18..=22 {
        for x in 18..=22 {
            b.grid.data[(y * size + x) as usize] = 100;
        }
    }
    let mut core = PlannerCore::new(b, cfg());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.5, 2.5, 0.0), &cancel).expect("solve");

    let on_obstacle = pose(20.5 * RES, 20.5 * RES, 0.0);
    // tolerance 0 だと NoAction。
    let mut strict = cfg();
    strict.action_tolerance_cells = 0;
    let strict_follow = follow::make_controller(&strict, &core.build.actions);
    let strict_core = PlannerCore {
        build: core.build.clone(),
        cfg: strict,
        cached: core.cached.take(),
        patch: core.patch.take(),
        penalty: core.penalty.take(),
        repair: core.repair.take(),
        dirty: false,
        prefetch: None,
        solve_only: false,
        follow: strict_follow,
    };
    assert_eq!(strict_core.decide(on_obstacle), Decision::NoAction);
    // tolerance 4 (0.2m) なら近傍の行動を借りられる。
    let relaxed_follow = follow::make_controller(&cfg(), &strict_core.build.actions);
    let relaxed_core = PlannerCore {
        build: strict_core.build.clone(),
        cfg: cfg(),
        cached: strict_core.cached,
        patch: strict_core.patch,
        penalty: strict_core.penalty,
        repair: strict_core.repair,
        dirty: false,
        prefetch: None,
        solve_only: false,
        follow: relaxed_follow,
    };
    assert!(matches!(relaxed_core.decide(on_obstacle), Decision::Action { .. }));
}

#[test]
fn unreachable_goal_fails_rollout() {
    // ゴール周辺だけ厚壁で囲む (中は free のままゴールを置く)。
    let size = 64;
    let mut b = build(size);
    for y in 32..size {
        for x in 40..48 {
            b.grid.data[(y * size + x) as usize] = 100;
        }
    }
    for y in 40..48 {
        for x in 40..size {
            b.grid.data[(y * size + x) as usize] = 100;
        }
    }
    let mut core = PlannerCore::new(b, cfg());
    let cancel = AtomicBool::new(false);
    let err = core.plan(pose(0.6, 0.6, 0.0), pose(2.8, 2.8, 0.0), &cancel).unwrap_err();
    assert!(matches!(err, PlanError::Rollout(RolloutStatus::NoAction)), "{err:?}");
}

/// パッチの寸法は「ウィンドウ + 遷移到達距離」を必ず超えること
/// (凍結境界が成り立つ条件そのもの)。粗いセルでも成り立つ。
#[test]
fn patch_geometry_covers_the_transition_reach() {
    for res in [0.05, 0.15, 0.25, 0.5] {
        let mut b = build(8);
        b.grid.resolution = res;
        // 津田沼構成と同じく歩幅をセルサイズに比例させた場合も見る。
        let k = res / 0.05;
        b.actions = actions()
            .into_iter()
            .enumerate()
            .map(|(i, a)| Action::new(&a.name, a.delta_fw * k, a.delta_rot, i as i32))
            .collect();
        let p = new_patch(&b).unwrap_or_else(|e| panic!("res={res}: {e}"));
        assert!(
            p.vi.local_ixy_range + p.reach < p.half,
            "res={res}: window {} + reach {} must fit in half {}",
            p.vi.local_ixy_range,
            p.reach,
            p.half
        );
    }
}

// ──────────────────────────────────────────────────────────────────────
// 狭域 → 広域のフィードバック (sweep_global)
// ──────────────────────────────────────────────────────────────────────

/// 横一本の通路だけが自由な地図。迂回路が無いので、通路を塞げばその西側
/// (ゴールと反対側) の価値は必ず上がる。
fn corridor(size: i32, y_lo: i32, y_hi: i32) -> BuildParams {
    let mut b = build(size);
    for iy in 0..size {
        for ix in 0..size {
            if iy < y_lo || iy > y_hi {
                b.grid.data[(iy * size + ix) as usize] = 100;
            }
        }
    }
    b
}

/// 全域掃きを `n` 回まわす (カーソルは持ち回す = ロックを手放しながら進める形)。
fn sweep_n(core: &mut PlannerCore, cur: &mut SweepCursor, n: usize) {
    for _ in 0..n {
        while !core.sweep_global(cur, usize::MAX).1 {}
    }
}

/// **この機能の本題。** 狭域が注入した local_penalty は、ローカル精密化だけでは
/// ウィンドウ (±1m) の外へ出ない。全域掃きを通して初めて広域の価値関数が動く。
///
/// 通路の真ん中を塞ぎ、ウィンドウの外にある西側のセルの価値を 3 時点で見る:
/// solve 直後 → 局所精密化の後 (変わらない) → 全域掃きの後 (上がる)。
///
/// **1 掃きで届く**ことを見ている。この地図での実測は 1 掃き目で +12%、30 掃きで
/// ほぼ収束、数値的な完全収束 (Δ=0) は約 80 掃き。運用上は収束を待つ必要はない —
/// 掃くたび不動点へ単調に近づき、経路が変わるのは遥かに手前なので、背景掃きは
/// 予算内で回し続ければよい (`sweep_global` の呼び出し規約)。
#[test]
fn local_penalty_reaches_the_global_field_only_after_a_global_sweep() {
    // 通路は iy 20..=30 (0.55m 幅)、ゴールは東の端。
    let mut core = PlannerCore::new(corridor(64, 20, 30), cfg());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.8, 1.25, 0.0), &cancel).expect("solve");

    let robot = pose(1.5, 1.25, 0.0); // セル (30, 25)
    // ウィンドウは ±20 セル = 10..=50。西の観測点はその外に取る。
    let west = {
        let vi = core.local().unwrap();
        pose_to_cell(&vi.base, 0.3, 1.25, 0.0) // セル (6, 25)
    };
    let value_at = |core: &PlannerCore, (ix, iy, it): (i32, i32, i32)| {
        let vi = core.local().unwrap();
        vi.base.states[vi.base.to_index(ix, iy, it) as usize].total_cost
    };

    let before = value_at(&core, west);
    assert!(before < MAX_COST, "西の観測点はゴールへ到達可能であること");
    assert!(!core.is_dirty(), "solve 直後は伝播させる仕事が無いこと");

    // 通路の断面を塞ぐ。1 本のビームでは ±2 セルしか立たないので、向きを
    // 振って 0.55m 幅を覆う (set_local_cost はビーム方向を pose の yaw で読む)。
    core.set_window(robot);
    for k in -3..=3 {
        let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.55] };
        core.observe_scan(&scan, pose(robot.x, robot.y, k as f64 * 0.12));
    }
    core.refine_for(Duration::from_secs(5));

    // 局所精密化はウィンドウの外を掃かないので、西側は動いていない。
    assert_eq!(
        value_at(&core, west),
        before,
        "ローカル精密化はウィンドウ (±1m) の外へ伝播しないこと"
    );
    assert!(core.is_dirty(), "狭域が場を動かしたら印が立つこと");

    // 1 掃きで広域に届くこと。
    let mut cur = SweepCursor::default();
    sweep_n(&mut core, &mut cur, 1);
    let after_one = value_at(&core, west);
    assert!(
        after_one > before,
        "1 掃きで広域の価値が上がっていること: before={before}, after={after_one}"
    );

    // 掃くほど不動点へ近づく (単調)。
    sweep_n(&mut core, &mut cur, 29);
    let after_thirty = value_at(&core, west);
    assert!(
        after_thirty > after_one,
        "掃くほど不動点へ近づくこと: 1 掃き={after_one}, 30 掃き={after_thirty}"
    );
}

/// 新しい不動点に達したら印が落ちて掃きが止まること (背景スレッドが収束後も
/// CPU を焼き続けないための性質)。上の本題と同じ状況を小さい地図で見る。
#[test]
fn sweeping_stops_once_the_field_settles() {
    let mut core = PlannerCore::new(corridor(24, 8, 14), cfg());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(1.0, 0.55, 0.0), &cancel).expect("solve");

    let robot = pose(0.5, 0.55, 0.0);
    core.set_window(robot);
    let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.3] };
    core.observe_scan(&scan, robot);
    core.refine_for(Duration::from_secs(5));
    assert!(core.is_dirty());

    let mut cur = SweepCursor::default();
    let mut sweeps = 0;
    while core.is_dirty() && sweeps < 500 {
        if core.sweep_global(&mut cur, usize::MAX).1 {
            sweeps += 1;
        }
    }
    assert!(!core.is_dirty(), "新しい不動点に達したら印は落ちること");
    assert!(sweeps > 0 && sweeps < 500, "有限回で収束すること (sweeps={sweeps})");
}

/// カーソルを持ち回してチャンクに切っても、1 掃きの結果は一気に掃いたときと
/// 同じになること (ロックを手放しながら進めるための性質)。
#[test]
fn chunked_sweeping_covers_the_same_cells_as_one_pass() {
    let goal = pose(2.0, 2.0, 0.0);
    let cancel = AtomicBool::new(false);

    let mut whole = PlannerCore::new(build(48), cfg());
    whole.prepare_goal(goal, &cancel).expect("solve");
    let mut cur = SweepCursor::default();
    let (_, done) = whole.sweep_global(&mut cur, usize::MAX);
    assert!(done, "max_cells が全体以上なら 1 回で掃き終わること");

    let mut chunked = PlannerCore::new(build(48), cfg());
    chunked.prepare_goal(goal, &cancel).expect("solve");
    let mut cur = SweepCursor::default();
    let mut chunks = 0;
    loop {
        let (_, done) = chunked.sweep_global(&mut cur, 1000);
        chunks += 1;
        if done {
            break;
        }
        assert!(chunks < 10_000, "チャンク掃きが終わらない");
    }
    assert!(chunks > 1, "1000 セル刻みなら複数チャンクに分かれること");

    let (a, b) = (whole.local().unwrap(), chunked.local().unwrap());
    assert_eq!(
        a.base.states.iter().map(|s| s.total_cost).collect::<Vec<_>>(),
        b.base.states.iter().map(|s| s.total_cost).collect::<Vec<_>>(),
        "チャンクに切っても 1 掃きの結果は変わらないこと"
    );
}

/// 1 掃き終わるごとに掃き順を次へ回すこと (伝播方向が偏らないように)。
#[test]
fn sweeps_rotate_through_the_sweep_orders() {
    let mut core = PlannerCore::new(build(32), cfg());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(1.0, 1.0, 0.0), &cancel).expect("solve");
    let orders = core.local().unwrap().base.sweep_orders.len();
    assert!(orders > 1);

    let mut cur = SweepCursor::default();
    for i in 0..orders {
        assert_eq!(cur.order, i, "掃き順は 1 掃きごとに進むこと");
        let (_, done) = core.sweep_global(&mut cur, usize::MAX);
        assert!(done);
    }
    assert_eq!(cur.order, 0, "一周したら戻ること");
}

/// 修復を待ち行列が空になるまで進める。戻り値は消費したタイル訪問数。
fn repair_until_settled(core: &mut PlannerCore, max_visits: usize) -> usize {
    let mut cur = SweepCursor::default();
    let mut visits = 0;
    while core.is_dirty() && visits < max_visits {
        core.sweep_global(&mut cur, usize::MAX);
        visits += 1;
    }
    assert!(!core.is_dirty(), "{max_visits} 訪問では修復が終わらない");
    visits
}

/// **タイル修復の本題。** compact の全域伝播が、密経路の全域掃きと**同じ**場に
/// 落ち着くこと。
///
/// 通路を塞ぎ、ウィンドウ (±1m) の外にある西側のセルで比べる。ここは
/// `commit_window` が届かない場所なので、値が一致するのはタイル修復が伝播を
/// 担っているときだけ。両者とも Δ=0 まで回してから比べる (同じ Bellman 作用素の
/// 不動点は 1 つなので、掃き順が違っても同じ値になる)。
#[test]
fn compact_global_propagation_matches_the_dense_global_sweep() {
    let goal = pose(2.8, 1.25, 0.0);
    let robot = pose(1.5, 1.25, 0.0);
    let cancel = AtomicBool::new(false);
    // 通路の断面を塞ぐスキャン (1 ビームでは ±2 セルしか立たないので向きを振る)。
    let block = |core: &mut PlannerCore| {
        core.set_window(robot);
        for k in -3..=3 {
            let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.55] };
            core.observe_scan(&scan, pose(robot.x, robot.y, k as f64 * 0.12));
        }
        core.refine_for(Duration::from_secs(5));
    };

    let mut dense = PlannerCore::new(corridor(64, 20, 30), cfg());
    dense.prepare_goal(goal, &cancel).expect("dense solve");
    // ウィンドウは ±20 セル = 10..=50。西の観測点はその外に取る。
    let west = {
        let vi = dense.local().unwrap();
        pose_to_cell(&vi.base, 0.3, 1.25, 0.0) // セル (6, 25)
    };
    let before = {
        let vi = dense.local().unwrap();
        vi.base.states[vi.base.to_index(west.0, west.1, west.2) as usize].total_cost
    };
    block(&mut dense);
    let mut cur = SweepCursor::default();
    let mut sweeps = 0;
    while dense.is_dirty() && sweeps < 400 {
        if dense.sweep_global(&mut cur, usize::MAX).1 {
            sweeps += 1;
        }
    }
    assert!(!dense.is_dirty(), "密経路が収束すること");
    let dense_value = {
        let vi = dense.local().unwrap();
        vi.base.states[vi.base.to_index(west.0, west.1, west.2) as usize].total_cost
    };
    assert!(dense_value > before, "塞いだら西側の値は上がること");

    let mut compact = PlannerCore::new(corridor(64, 20, 30), cfg_compact());
    compact.prepare_goal(goal, &cancel).expect("compact solve");
    block(&mut compact);
    assert!(compact.is_dirty(), "窓を書き戻したら伝播の仕事が積まれること");
    let visits = repair_until_settled(&mut compact, 8_000);
    let compact_value = {
        let Some(CachedGoal { field: Field::Compact(f), .. }) = compact.cached.as_ref() else {
            panic!("compact field expected");
        };
        f.sink.read(f.orig(west.0, west.1, west.2)).0
    };

    assert_eq!(
        compact_value, dense_value,
        "ウィンドウの外 (セル {},{},{}) で密の全域掃きと同じ値になること \
         (タイル訪問 {visits} 回 / 密は {sweeps} 掃き)",
        west.0, west.1, west.2
    );
}

/// 通路を**幅いっぱい**に塞いだら、遠方 (窓の外) の値がその分だけ上がること。
///
/// 上のパリティ試験は「compact が密と同じ不動点に落ちる」ことしか見ていない。
/// 実機で確かめたいのはその手前の「不動点が実用的な量だけ動く」ほうなので、
/// 追従 1 tick + 修復数枚を交互に回す (掃きスレッドの duty そのもの) 形で
/// 遠方の値を追う。実測 13 → 38 ステップ、119 ラウンド (= 12 秒相当) /
/// タイル訪問 358 回で落ち着く。
///
/// **塞ぎ方で桁が変わる**。通路の一部だけを塞ぐと迂回できてしまい遠方の値は
/// ほとんど動かない (別測定: 幅 2m の通路を幅 0.4m だけ塞いで +0.75 ステップ
/// = `cost_drawing_threshold: 60` の色 1 段ぶん)。効いていないように見えても
/// たいていはこれで、伝播の不具合ではない。
#[test]
fn a_full_width_block_raises_the_value_far_outside_the_window() {
    const N: i32 = 100;
    let mut core = PlannerCore::new(corridor(N, 30, 50), cfg_compact());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(4.5, 2.0, 0.0), &cancel).expect("solve");

    let robot = pose(3.0, 2.0, 0.0);
    // 遠方の観測点はロボットの 2.5m 西 = 窓 (±1m = ±20 セル) の 30 セル外。
    let (fx, fy) = (10i32, 40i32);
    let read = |core: &PlannerCore| {
        let Some(CachedGoal { field: Field::Compact(f), .. }) = core.cached.as_ref() else {
            panic!("compact field expected");
        };
        f.sink.read(f.orig(fx, fy, 0)).0
    };
    let before = read(&core);

    // x = 3.55 の縦線を通路幅いっぱいに塗る。
    let inc = 0.05;
    let a0 = -0.9;
    let ranges: Vec<f64> = (0..37).map(|i| 0.55 / (a0 + inc * i as f64).cos()).collect();
    let scan = LaserScan { angle_min: a0, angle_increment: inc, ranges };

    let mut cur = SweepCursor::default();
    let mut visits = 0usize;
    let mut settled = None;
    for round in 0..300 {
        core.set_window(robot);
        core.observe_scan(&scan, robot);
        core.refine_for(Duration::from_secs(5));
        // 掃きスレッドの 1 周期ぶん (budget 20ms / idle 60ms で数枚)。
        for _ in 0..3 {
            if !core.is_dirty() {
                break;
            }
            core.sweep_global(&mut cur, usize::MAX);
            visits += 1;
        }
        if !core.is_dirty() {
            settled = Some(round);
            break;
        }
    }
    let after = read(&core);
    let steps = |v: u64| v / 262144;
    assert!(
        settled.is_some(),
        "300 ラウンド (= 30 秒相当) で伝播が落ち着くこと (訪問 {visits} 回)"
    );
    // 塞ぎの penalty (2048 ステップ) を丸ごと払うわけではない — 帯の縁を
    // かすめる経路が残るので、その差額ぶんだけ上がる。
    assert!(
        steps(after) >= steps(before) + 10,
        "遠方の値が実用的な量だけ上がること ({} -> {} ステップ、{settled:?} ラウンド / 訪問 {visits} 回)",
        steps(before),
        steps(after),
    );
}

/// 修復と追従パッチが互いを上書きし合って**終わらなくならない**こと。
///
/// パッチは sink の写しを凍結して持っているので、修復がその footprint を
/// 書き換えたら無効化して起こし直させる必要がある。しないと「パッチが古い
/// 凍結値から計算 → `commit_window` が書き戻す → 修復がまた直す」で
/// `is_dirty` が永久に落ちない (値はどちらも正しいので、症状は掃きが
/// 終わらないことだけ = 気付きにくい)。
///
/// 追従 1 tick と修復の空になるまでを交互に回し、数ラウンドで「追従 tick が
/// 何も動かさない」ところへ落ち着くのを見る。
#[test]
fn compact_repair_and_the_follow_patch_settle_together() {
    let mut core = PlannerCore::new(corridor(64, 20, 30), cfg_compact());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.8, 1.25, 0.0), &cancel).expect("solve");

    let robot = pose(1.5, 1.25, 0.0);
    core.set_window(robot);
    for k in -3..=3 {
        let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.55] };
        core.observe_scan(&scan, pose(robot.x, robot.y, k as f64 * 0.12));
    }
    core.refine_for(Duration::from_secs(5));

    let mut rounds = 0;
    let quiet = loop {
        repair_until_settled(&mut core, 8_000);
        // 追従 1 tick 相当 (置き直しが要ればここで起き直す)。
        core.set_window(robot);
        core.refine_for(Duration::from_secs(5));
        rounds += 1;
        if !core.is_dirty() {
            break true;
        }
        if rounds >= 8 {
            break false;
        }
    };
    assert!(quiet, "追従 tick と修復が同じ不動点に落ち着くこと ({rounds} ラウンド回した)");
}

/// 修復タイルの halo が遷移の届く距離をちゃんと覆っていること (足りないと
/// interior の縁だけが MAX_COST を見て値が静かにずれる)、および halo に対して
/// 割の合う interior になっていること。
#[test]
fn repair_tile_geometry_covers_the_transition_reach() {
    let b = build(64);
    let patch = new_patch(&b).expect("patch");
    let r = new_repair(&b, patch.reach).expect("repair tile");
    assert_eq!(r.halo, patch.reach.max(1), "halo は遷移到達距離そのもの");
    assert!(r.halo >= transition_reach(&r.vi), "halo が凍結境界として足りること");
    assert!(r.interior >= r.halo, "interior が halo を下回らないこと");
    assert_eq!(r.vi.cell_num_x, r.interior + 2 * r.halo);
    // タイル格子は地図を覆い切ること。
    assert!(r.tnx * r.interior >= b.grid.width);
    assert!(r.tny * r.interior >= b.grid.height);
}

/// ゴールを解き直したら待ち行列も空にすること (前のゴールの場を修復しても
/// 意味がない。密経路の `dirty` を落とすのと同じ寿命)。
#[test]
fn resolving_a_new_goal_clears_the_repair_queue() {
    let mut core = PlannerCore::new(build(64), cfg_compact());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.5, 1.0, 0.0), &cancel).expect("solve");

    let robot = pose(1.0, 1.0, 0.0);
    core.set_window(robot);
    let scan = LaserScan { angle_min: 0.0, angle_increment: 0.0, ranges: vec![0.5] };
    core.observe_scan(&scan, robot);
    core.refine_passes(5);
    assert!(!core.repair.as_ref().unwrap().queue.is_empty(), "伝播の仕事が積まれること");

    core.prepare_goal(pose(0.5, 2.5, 0.0), &cancel).expect("re-solve");
    assert!(core.repair.as_ref().unwrap().queue.is_empty());
    assert!(!core.is_dirty());
}

/// `global_sweep: false` なら修復タイル (数 MB) を確保しないこと。
#[test]
fn repair_tile_is_not_allocated_when_the_sweep_is_off() {
    let mut cfg = cfg_compact();
    cfg.global_sweep = false;
    let mut core = PlannerCore::new(build(64), cfg);
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("solve");
    assert!(core.repair.is_none());

    // 掃きを呼んでも何もせず、印だけ落として返ること (呼び出し側が空回りしない)。
    core.dirty = true;
    let mut cur = SweepCursor::default();
    assert_eq!(core.sweep_global(&mut cur, usize::MAX), (0, true));
    assert!(!core.is_dirty());
}

/// ゴールを解き直したら印は持ち越さないこと (最初の 1 掃きが無駄に回らない)。
#[test]
fn resolving_a_new_goal_clears_the_pending_propagation() {
    let mut core = PlannerCore::new(build(48), cfg());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(pose(2.0, 2.0, 0.0), &cancel).expect("solve");
    core.dirty = true;
    core.prepare_goal(pose(0.5, 0.5, 0.0), &cancel).expect("re-solve");
    assert!(!core.is_dirty());
}

// ──────────────────────────────────────────────────────────────────────────────
// ウェイポイントの先読み (core::prefetch)
//
// ワーカーは別スレッドで走るが、`adopt` が「解き終わるまで待つ」ので待ち合わせは
// 要らない (sleep で同期するテストにはしない)。
// ──────────────────────────────────────────────────────────────────────────────

/// 先読みの本題: 2 点目の solve が**走行中に済んでいる**こと。
/// 巡回で点が変わるたびに止まっていた solve 1 回ぶんがこれで消える。
#[test]
fn the_next_waypoint_is_adopted_without_solving_again() {
    let wps = vec![pose(1.0, 1.0, 0.0), pose(2.0, 2.0, 0.0)];
    let pf = Prefetcher::spawn(build(64), cfg());
    pf.set_waypoints(wps.clone());
    let mut core = PlannerCore::new(build(64), cfg()).with_prefetch(pf);
    let cancel = AtomicBool::new(false);

    // 1 点目。先読みはまだ何も持っていないので自分で解く。解き終わった時点で
    // 2 点目が注文される。
    let s1 = core.prepare_goal(wps[0], &cancel).expect("solve the first waypoint");
    assert!(s1.solved_now && s1.iters > 0 && !s1.adopted);

    // 2 点目。先読みワーカーが解いた場をそのまま受け取る (この呼び出しは
    // 1 イテレーションも回さない)。
    let s2 = core.prepare_goal(wps[1], &cancel).expect("adopt the second waypoint");
    assert!(s2.adopted, "2 点目は先読みから受け取ること");
    assert_eq!(s2.iters, 0, "solve は先読みワーカーが済ませているはず");
    assert!(s2.solved_now, "場が入れ替わったのはキャッシュヒットではない");
    assert!(core.is_cached_goal(wps[1]));
}

/// 受け取った場が「自分で解いた場」と同じに扱われること。とくに compact の
/// 追従パッチは solve の中ではなく外で作るようにしてあるので、**1 点目から
/// 受け取っても**追従できる (パッチが無いと decide は NoAction しか返さない)。
#[test]
fn a_goal_adopted_as_the_first_one_can_still_be_followed() {
    let wps = vec![pose(1.0, 1.0, 0.0), pose(2.0, 2.0, 0.0)];
    let pf = Prefetcher::spawn(build(64), cfg_compact());
    pf.set_waypoints(wps.clone());
    let cancel = AtomicBool::new(false);

    // 注文だけさせる核 (ここでは 1 点目を解く)。
    let mut warm = PlannerCore::new(build(64), cfg_compact()).with_prefetch(pf.clone());
    warm.prepare_goal(wps[0], &cancel).expect("solve the first waypoint");

    // まっさらな核が 2 点目を受け取る = solve を一度も通らない。
    let mut fresh = PlannerCore::new(build(64), cfg_compact()).with_prefetch(pf);
    let s = fresh.prepare_goal(wps[1], &cancel).expect("adopt");
    assert!(s.adopted);
    assert!(fresh.patch.is_some(), "solve を通らなくても追従パッチは要る");
    assert!(fresh.repair.is_some(), "全域伝播の作業場も同じ");
    assert!(!fresh.is_dirty());

    let start = pose(1.0, 1.0, 0.0);
    fresh.set_window(start);
    assert!(
        matches!(fresh.decide(start), Decision::Action { .. }),
        "受け取った場の上で方策が読めること"
    );
}

/// 並びに無いゴールが来たら、先読みを待たずに自分で解くこと (待つと、巡回の
/// 途中に単発ゴールを 1 つ挟んだだけで先読み 1 回ぶん止まる)。
#[test]
fn a_goal_outside_the_list_is_solved_without_waiting_for_the_prefetch() {
    let wps = vec![pose(1.0, 1.0, 0.0), pose(2.0, 2.0, 0.0)];
    let pf = Prefetcher::spawn(build(64), cfg());
    pf.set_waypoints(wps.clone());
    let mut core = PlannerCore::new(build(64), cfg()).with_prefetch(pf);
    let cancel = AtomicBool::new(false);
    core.prepare_goal(wps[0], &cancel).expect("solve the first waypoint");

    // 2 点目を先読みしている最中に、並びに無いゴールが来る。
    let elsewhere = pose(0.5, 2.5, 0.0);
    let s = core.prepare_goal(elsewhere, &cancel).expect("solve");
    assert!(!s.adopted, "別ゴールの先読みを受け取ってはいけない");
    assert!(s.iters > 0, "自分で解くこと");
    assert!(core.is_cached_goal(elsewhere));
}

/// 巡回中の「いまの点」を計画し直しても、次の点の先読みは走り続けること。
///
/// BT は追従中も 1Hz で `compute_path_to_pose` を投げ直し、復帰行動でも同じ点を
/// 計画し直す。ここで先読みが取り消されると、次の点に着いたときに solve が丸ごと
/// 1 回走る = この機能が消したはずの停止がそのまま戻る。取り消しが起きないのは
/// 同じ点ならキャッシュヒットで `adopt` まで来ないからで、`adopt` の doc に書いた
/// 「進行中の先読みは常にキャッシュ中のゴールの次」という対応がその根拠。
#[test]
fn replanning_the_current_goal_leaves_the_prefetch_running() {
    let wps = vec![pose(1.0, 1.0, 0.0), pose(2.0, 2.0, 0.0)];
    let pf = Prefetcher::spawn(build(64), cfg());
    pf.set_waypoints(wps.clone());
    let mut core = PlannerCore::new(build(64), cfg()).with_prefetch(pf.clone());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(wps[0], &cancel).expect("solve the first waypoint");

    // BT のリプラン相当。キャッシュヒットなので solve もしない。
    let again = core.prepare_goal(wps[0], &cancel).expect("replan");
    assert!(!again.solved_now && !again.adopted);

    // 2 点目の先読みは生きたまま (注文中か、もう解けているか)。
    let alive = pf.pending().is_some()
        || core.prepare_goal(wps[1], &cancel).expect("adopt").adopted;
    assert!(alive, "いまの点を計画し直しただけで次の点の先読みを捨ててはいけない");
}

/// 最後の点まで来たら注文しない (並びの外を勝手に解かない)。
#[test]
fn the_last_waypoint_does_not_queue_another_prefetch() {
    let wps = vec![pose(1.0, 1.0, 0.0)];
    let pf = Prefetcher::spawn(build(64), cfg());
    pf.set_waypoints(wps.clone());
    let mut core = PlannerCore::new(build(64), cfg()).with_prefetch(pf.clone());
    let cancel = AtomicBool::new(false);
    core.prepare_goal(wps[0], &cancel).expect("solve");
    assert!(pf.pending().is_none(), "最後の点の次は無い");
}

/// 先読みは価値関数を 2 つ同時に生かす。compact の確定出力は solve ごとに
/// 別ディレクトリへ置くこと — 固定ファイル名のままだと、後から解くほうが
/// `truncate` で先の場のファイルを潰す (mmap 中に長さが変わる)。
#[test]
fn two_live_compact_fields_never_share_a_sink_directory() {
    let dir = std::env::temp_dir().join("vi_planner_sink_gen_test");
    let _ = std::fs::remove_dir_all(&dir);
    let gen = Arc::new(AtomicU64::new(0));
    let cfg_gen = || PlanConfig {
        compact_sink_dir: Some(dir.clone()),
        compact_sink_gen: Some(Arc::clone(&gen)),
        ..cfg_compact()
    };
    let cancel = AtomicBool::new(false);

    let mut a = PlannerCore::new(build(48), cfg_gen());
    a.prepare_goal(pose(1.0, 1.0, 0.0), &cancel).expect("solve a");
    let before = a.value_grid(60).expect("grid a");

    // 2 本目。ここで a の sink を潰すようだと、先読みは使いものにならない。
    let mut b = PlannerCore::new(build(48), cfg_gen());
    b.prepare_goal(pose(2.0, 0.5, 0.0), &cancel).expect("solve b");

    assert_eq!(before.data, a.value_grid(60).expect("grid a again").data, "a の場が無傷であること");
    let gens = std::fs::read_dir(&dir)
        .expect("sink dir")
        .flatten()
        .filter(|e| e.file_name().to_string_lossy().starts_with("gen"))
        .count();
    assert_eq!(gens, 2, "solve ごとに 1 つ");

    drop(a);
    drop(b);
    let _ = std::fs::remove_dir_all(&dir);
}

/// `main.rs` は核を `Arc<Mutex<PlannerCore>>` にしてスレッドへ配る。**その形は
/// ホストでは組めない** (rclrs が colcon 経由でしかリンクしない) ので、必要な
/// 境界だけここで固定する。先読みを足したときに壊れるならこのテストで落ちる。
#[test]
fn the_core_and_the_prefetcher_cross_threads() {
    fn send<T: Send>() {}
    fn send_sync<T: Send + Sync>() {}
    // `Mutex<T>: Sync` は `T: Send` で足りる。核そのものは Sync ではない
    // (sink が `dyn CompactSink + Send` なので) — 共有はロック越しだけ。
    send::<PlannerCore>();
    // 先読みの取っ手は核の中に入ったまま別スレッドへ渡るので、両方が要る。
    send_sync::<Prefetcher>();
}

// ──────────────────────────────────────────────────────────────────────────────
// 走り出しの短縮 (core::PlanConfig::early_start)
//
// 判定は密経路も compact 経路も同じ (`reaches_goal` = plan が返す経路の作り方
// そのもの) なので、両者で見るのは「打ち切ったか」ではなく**打ち切った場が
// 使えるか**にしてある。
// ──────────────────────────────────────────────────────────────────────────────

fn cfg_early() -> PlanConfig {
    PlanConfig { early_start: true, ..cfg() }
}

/// 密経路の本題: 起点からゴールまで方策が繋がった時点で solve が止まり、
/// **その場のまま**経路が引けて追従もできること。
#[test]
fn early_start_cuts_the_dense_solve_once_the_path_exists() {
    let goal = pose(0.5, 0.5, 0.0);
    let start = pose(1.5, 1.5, 0.0);
    let cancel = AtomicBool::new(false);

    let mut full = PlannerCore::new(build(64), cfg());
    let (_, sf) = full.plan(start, goal, &cancel).expect("full plan");
    assert!(!sf.truncated);

    let mut early = PlannerCore::new(build(64), cfg_early());
    let (path, se) = early.plan(start, goal, &cancel).expect("early plan");
    assert!(se.truncated, "起点まで繋がった時点で止まること");
    assert!(se.iters < sf.iters, "打ち切ったぶん短いこと: {} vs {}", se.iters, sf.iters);
    assert!(path.len() > 2);

    // 打ち切った場でも追従の判断が下せる (方策は収束前でも書き戻されている)。
    early.set_window(start);
    assert!(matches!(early.decide(start), Decision::Action { .. }));
}

/// 打ち切った密の場には「まだ伝播させる仕事がある」印を付ける。走りながら
/// 全域掃きが最後まで詰めるための唯一の判断材料がこれ (`global_sweep: true`)。
#[test]
fn a_truncated_dense_field_is_left_dirty_for_the_global_sweep() {
    let cancel = AtomicBool::new(false);
    let mut core = PlannerCore::new(build(64), cfg_early());
    let s = core
        .plan(pose(1.5, 1.5, 0.0), pose(0.5, 0.5, 0.0), &cancel)
        .expect("plan")
        .1;
    assert!(s.truncated);
    assert!(core.is_dirty(), "打ち切った場は掃きに渡すこと");

    // 収束まで解いた場は逆に印を持たない (最初の 1 掃きが無駄に回る)。
    let mut done = PlannerCore::new(build(64), cfg());
    done.plan(pose(1.5, 1.5, 0.0), pose(0.5, 0.5, 0.0), &cancel).expect("plan");
    assert!(!done.is_dirty());
}

/// compact 経路: 打ち切った sink には経路に要る列だけが載り、遠くは未確定
/// (`MAX_COST`) のまま。**それでも載っている値は最後まで解いたときと同じ** —
/// finalize が値の昇順に進むので、確定した列は以後動かない。
///
/// 地図が小さいと値域が丸ごと 1 バンドに収まって波が 2 つで終わる (= 打ち切る
/// 隙が無い) ので、ここだけ広めの地図を使う。
#[test]
fn early_start_cuts_the_compact_solve_and_leaves_the_far_side_unfinalized() {
    let goal = pose(1.0, 1.0, 0.0);
    let start = pose(4.0, 4.0, 0.0);
    let cancel = AtomicBool::new(false);

    let mut core = PlannerCore::new(build(256), cfg_early_compact());
    let (path, s) = core.plan(start, goal, &cancel).expect("early plan");
    assert!(s.truncated, "波が残っているうちに止まること");
    assert!(path.len() > 2);

    // 起点まわりは確定済み、地図の反対の隅はまだ。**この 2 つは走り出す前に見る** —
    // 走り出すと窓の書き戻し (`commit_window`) とタイル修復が未確定域を埋めていく。
    let far = pose(12.0, 12.0, 0.0);
    assert!(compact_value_at(&core, start) < MAX_COST, "起点は確定していること");
    assert_eq!(compact_value_at(&core, far), MAX_COST, "遠くは未確定のままのはず");

    // **本題**: 打ち切った場のままゴール圏まで走り切れること (既定の solver は
    // compact なので、実機で通るのはこの経路)。窓は毎 tick 未確定域と確定域の
    // 境目をまたぐので、そこで NoAction になるならここで落ちる。
    let (mut x, mut y, mut yaw) = (start.x, start.y, start.yaw_rad);
    for _ in 0..2000 {
        let p = pose(x, y, yaw);
        core.set_window(p);
        core.refine_passes(1);
        match core.decide(p) {
            Decision::Goal => {
                assert!(core.goal_distance(x, y).unwrap() <= 0.3);
                return;
            }
            Decision::Action { fw, rot_deg, .. } => {
                x += fw * yaw.cos();
                y += fw * yaw.sin();
                yaw += rot_deg.to_radians();
            }
            Decision::NoAction => panic!("no action at ({x:.2}, {y:.2}) on the truncated field"),
        }
    }
    panic!("did not reach the goal in 2000 steps");
}

fn cfg_early_compact() -> PlanConfig {
    PlanConfig { early_start: true, ..cfg_compact() }
}

/// compact の場の `(x, y, θ=0)` の値。未確定の列は `MAX_COST` を返す。
fn compact_value_at(core: &PlannerCore, p: PoseView) -> u64 {
    let Some(CachedGoal { field: Field::Compact(f), .. }) = core.cached.as_ref() else {
        panic!("compact field expected");
    };
    let ix = ((p.x - f.origin.0) / f.resolution).floor() as i32;
    let iy = ((p.y - f.origin.1) / f.resolution).floor() as i32;
    f.sink.read(f.orig(ix, iy, 0)).0
}

/// 起点を渡さない solve は `early_start` でも最後まで解く。先読みのワーカーが
/// 通るのがこの道で、打ち切った場を渡されると困る (次の点に着く頃には機体は
/// 起点にいない)。
#[test]
fn a_solve_without_an_anchor_is_never_cut_short() {
    let cancel = AtomicBool::new(false);
    let mut core = PlannerCore::new(build(64), cfg_early());
    let s = core.prepare_goal(pose(0.5, 0.5, 0.0), &cancel).expect("solve");
    assert!(s.solved_now && !s.truncated);
}

/// 打ち切りの唯一の逃げ道: 経路の外の未確定領域から計画を頼まれたら、
/// **キャッシュを捨てて解き直す**。返しっぱなしにすると BT が投げ直しても
/// キャッシュヒットで同じ失敗を繰り返す。
#[test]
fn a_truncated_field_that_cannot_be_rolled_out_is_solved_again() {
    let goal = pose(0.3, 0.3, 0.0);
    let cancel = AtomicBool::new(false);
    // 打ち切りを観測するのはチャンクの切れ目なので、刻みを 1 にして「起点に
    // 届いた直後」で止める (既定の 16 だと、その 1 チャンクのあいだに波が
    // 地図の反対側まで行ってしまって未確定領域が残らない)。
    let mut core = PlannerCore::new(build(96), PlanConfig { solve_chunk: 1, ..cfg_early() });

    let s1 = core.plan(pose(0.9, 0.9, 0.0), goal, &cancel).expect("plan near").1;
    assert!(s1.truncated);

    // 打ち切った時点では届いていない、ゴールからもっと遠い起点。
    let (path, s2) = core.plan(pose(4.5, 4.5, 0.0), goal, &cancel).expect("plan far");
    assert!(s2.solved_now, "キャッシュを捨てて解き直すこと");
    assert!(!s2.truncated, "解き直しは打ち切らない");
    assert!(path.len() > 2);
    assert!(!core.discard_truncated(), "解き直した場に打ち切りの印は残らない");
}

/// 収束済みの場は捨てない (`discard_truncated` は打ち切った場だけの逃げ道)。
#[test]
fn discard_truncated_leaves_a_converged_field_alone() {
    let goal = pose(2.0, 2.0, 0.0);
    let cancel = AtomicBool::new(false);
    let mut core = PlannerCore::new(build(64), cfg());
    core.prepare_goal(goal, &cancel).expect("solve");
    assert!(!core.discard_truncated());
    assert!(core.is_cached_goal(goal), "捨てられていないこと");
}

/// 能動的再定位: 多目標場が解けて QMDP が乗ること。キャッシュは本来のゴールと
/// 一致しない (復帰後の最初の要求が解き直す)。compact 構成は Unsupported。
#[test]
fn prepare_reloc_goal_serves_qmdp_toward_targets() {
    let cancel = AtomicBool::new(false);
    let mut core = PlannerCore::new(build(64), cfg());
    let targets = [(0.8, 0.8), (2.4, 2.4)];
    let stats = core.prepare_reloc_goal(&targets, &cancel).expect("reloc field");
    assert!(stats.solved_now);
    assert!(!core.is_cached_goal(pose(1.6, 1.6, 0.0)), "本来のゴールとは不一致のはず");

    // 2 仮説 QMDP: どちらの仮説にも行動が出る (多目標場の上の走行)。
    let hyps = [(pose(0.5, 0.5, 0.0), 0.5), (pose(2.7, 2.7, 0.0), 0.5)];
    match core.decide_qmdp(&hyps) {
        Decision::Action { .. } => {}
        d => panic!("expected Action, got {d:?}"),
    }
    // 両仮説とも判別点圏内なら Goal (final_state は全 θ)。
    let at = [(pose(0.8, 0.8, 1.0), 0.5), (pose(2.4, 2.4, 2.0), 0.5)];
    assert_eq!(core.decide_qmdp(&at), Decision::Goal);

    let mut compact = PlannerCore::new(build(64), cfg_compact());
    assert!(matches!(
        compact.prepare_reloc_goal(&targets, &cancel),
        Err(PlanError::Unsupported(_))
    ));
}
