//! `value_iterator.rs` のユニットテスト。`#[path]` で value_iterator の
//! サブモジュールとして取り込まれる（`super` は value_iterator を指す）。
//! 本家ワーカー実行経路 (`value_iteration_worker` / `run_value_iteration` /
//! `finished`) のテストは `solvers/original/tests.rs` にある。

use super::*;

#[test]
fn to_index_layout() {
    // cell_num_x=4, cell_num_t=60。
    assert_eq!(to_index_raw(0, 0, 0, 4, 60), 0);
    assert_eq!(to_index_raw(0, 0, 5, 4, 60), 5);
    assert_eq!(to_index_raw(1, 0, 0, 4, 60), 60);
    assert_eq!(to_index_raw(0, 1, 0, 4, 60), 240);
}

#[test]
fn cell_delta_negative_correction() {
    // xy_res=0.05。x=-0.01 → |x|/res=0.2 → floor 0 → x<0 → -0-1 = -1。
    let (ix, _, _) = cell_delta(-0.01, 0.0, 0.0, 0.05, 6.0);
    assert_eq!(ix, -1);
    // x=0.06 → 1.2 → floor 1。
    let (ix2, _, _) = cell_delta(0.06, 0.0, 0.0, 0.05, 6.0);
    assert_eq!(ix2, 1);
}

#[test]
fn cell_delta_theta_absolute_not_normalized() {
    // t=366, t_res=6 → floor(61) = 61 (絶対、wrap しない)。
    let (_, _, it) = cell_delta(0.0, 0.0, 366.0, 0.05, 6.0);
    assert_eq!(it, 61);
}

#[test]
fn no_noise_negative_theta_normalized_once() {
    // from_t=10, rot=-20 → to_t=-10 → +360 = 350。
    let (_, _, to_t) = no_noise_state_transition(0.0, -20.0, 0.0, 0.0, 10.0);
    assert!((to_t - 350.0).abs() < 1e-9);
}

#[test]
fn no_noise_over_360_not_normalized() {
    // from_t=350, rot=20 → to_t=370 (>=360 は残す)。
    let (_, _, to_t) = no_noise_state_transition(0.0, 20.0, 0.0, 0.0, 350.0);
    assert!((to_t - 370.0).abs() < 1e-9);
}

#[test]
fn no_noise_forward_uses_cos_sin() {
    // fw=0.3, from_t=0 → to_x=0.3, to_y=0。
    let (to_x, to_y, _) = no_noise_state_transition(0.3, 0.0, 0.0, 0.0, 0.0);
    assert!((to_x - 0.3).abs() < 1e-9);
    assert!(to_y.abs() < 1e-9);
}

fn free_grid(w: i32, h: i32) -> OccupancyGrid {
    OccupancyGrid {
        width: w,
        height: h,
        resolution: 0.05,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: Default::default(),
        data: vec![0; (w * h) as usize],
    }
}

#[test]
fn set_map_occupancy_populates_states_and_transitions() {
    let actions = vec![Action::new("forward", 0.3, 0.0, 0)];
    let mut vi = ValueIterator::new(actions, 1);
    let map = free_grid(3, 2);
    vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);

    assert_eq!(vi.cell_num_x, 3);
    assert_eq!(vi.cell_num_y, 2);
    assert_eq!(vi.cell_num_t, 60);
    assert_eq!(vi.t_resolution, 6.0);
    assert_eq!(vi.states.len(), 3 * 2 * 60);
    // 各 action の θ ごとに遷移が生成されている。
    assert_eq!(vi.actions[0].state_transitions.len(), 60);
    let total: i64 = vi.actions[0].state_transitions[0]
        .iter()
        .map(|s| s.prob as i64)
        .sum();
    assert_eq!(total, super::PROB_BASE as i64);
}

#[test]
fn sweep_orders_structure() {
    let actions = vec![Action::new("forward", 0.3, 0.0, 0)];
    let mut vi = ValueIterator::new(actions, 1);
    let map = free_grid(2, 2);
    vi.set_map_with_occupancy_grid(&map, 4, 0.2, 30.0, 0.2, 10); // 小さい cell_num_t=4
    let total = (2 * 2 * 4) as usize;
    assert_eq!(vi.sweep_orders.len(), 6);
    assert_eq!(vi.sweep_orders[0].len(), total);
    assert_eq!(vi.sweep_orders[1].len(), total);
    // [2],[3] は逆順
    let rev0: Vec<i32> = vi.sweep_orders[0].iter().rev().cloned().collect();
    assert_eq!(vi.sweep_orders[2], rev0);
    // ★[4] = [0]全体 + [1]後半 (size = total + (total - half))
    let half = total / 2;
    assert_eq!(vi.sweep_orders[4].len(), total + (total - half));
    assert_eq!(&vi.sweep_orders[4][..total], &vi.sweep_orders[0][..]);
    assert_eq!(&vi.sweep_orders[4][total..], &vi.sweep_orders[1][half..]);
    // ★[5] = [1]前半
    assert_eq!(vi.sweep_orders[5], vi.sweep_orders[1][..half].to_vec());
}

#[test]
fn sweep_orders_idempotent() {
    let actions = vec![Action::new("forward", 0.3, 0.0, 0)];
    let mut vi = ValueIterator::new(actions, 1);
    let map = free_grid(2, 2);
    vi.set_map_with_occupancy_grid(&map, 4, 0.2, 30.0, 0.2, 10);
    let len_before = vi.sweep_orders.len();
    vi.set_sweep_orders(); // 2 回目は no-op
    assert_eq!(vi.sweep_orders.len(), len_before);
}

#[test]
fn prob_sum_equals_prob_base() {
    // 任意 action・θで prob 総和 = 64^3 = 262144 = PROB_BASE。
    let list = compute_theta_transitions(0.3, 0.0, 0, 0.05, 6.0);
    let total: i64 = list.iter().map(|s| s.prob as i64).sum();
    assert_eq!(total, super::PROB_BASE as i64);
}

#[test]
fn forward_theta0_moves_in_x() {
    // 前進 fw=0.3, θ=0, res=0.05 → 主に dix≈6, diy=0, dit=0。
    let list = compute_theta_transitions(0.3, 0.0, 0, 0.05, 6.0);
    // 最頻バケット (prob 最大) を確認。
    let top = list.iter().max_by_key(|s| s.prob).unwrap();
    assert_eq!(top.diy, 0);
    assert_eq!(top.dit, 0, "θ=0 の前進は絶対 θ=0");
    assert!(top.dix >= 5 && top.dix <= 6, "dix was {}", top.dix);
}

#[test]
fn rotation_dit_is_absolute_theta() {
    // 左回転 rot=+20, θ=0, t_res=6 → to_t≈20 → dit≈3 (絶対)、dix=diy=0。
    let list = compute_theta_transitions(0.0, 20.0, 0, 0.05, 6.0);
    let top = list.iter().max_by_key(|s| s.prob).unwrap();
    assert_eq!(top.dix, 0);
    assert_eq!(top.diy, 0);
    assert_eq!(top.dit, 3, "rot+20 → 絶対 θ index 3");
}

#[test]
fn rotation_dit_absolute_at_theta30() {
    // θ=30 (index 5, t_res=6 → θ_origin=30°), 左回転 +20 → to_t≈50 → dit≈8 (絶対)。
    let list = compute_theta_transitions(0.0, 20.0, 5, 0.05, 6.0);
    let top = list.iter().max_by_key(|s| s.prob).unwrap();
    assert_eq!(top.dit, 8, "θ_origin30 + rot20 = 50° → index 8 (絶対)");
}

fn mk_state(ix: i32, iy: i32, it: i32, free: bool, total: u64, penalty: u64) -> State {
    State {
        total_cost: total,
        penalty,
        local_penalty: 0,
        ix,
        iy,
        it,
        free,
        final_state: false,
        optimal_action: None,
    }
}

fn single_action(dix: i32, diy: i32, dit: i32, nt: usize) -> Action {
    let mut a = Action::new("a", 0.0, 0.0, 0);
    a.state_transitions = vec![Vec::new(); nt];
    for it in 0..nt {
        a.state_transitions[it].push(StateTransition::new(dix, diy, dit, super::PROB_BASE as i32));
    }
    a
}

#[test]
fn action_cost_deterministic_neighbor() {
    // 2x1 マップ、θ=0。dix=+1 で隣 (free, total=5*PROB_BASE, penalty=PROB_BASE)。
    // cost = (5*PB + PB)*PB >>18 = 6*PB。
    let nt = 1usize;
    let nx = 2;
    let ny = 1;
    let states = vec![
        mk_state(0, 0, 0, true, super::MAX_COST, super::PROB_BASE),
        mk_state(1, 0, 0, true, 5 * super::PROB_BASE, super::PROB_BASE),
    ];
    let a = single_action(1, 0, 0, nt);
    let s = states[0].clone();
    let c = super::action_cost_raw(&states, &a, &s, 0, nx, ny, nt as i32, &BeliefModel::default());
    assert_eq!(c, 6 * super::PROB_BASE);
}

#[test]
fn action_cost_out_of_map_returns_max() {
    let nt = 1usize;
    let states = vec![mk_state(0, 0, 0, true, 0, 0)];
    let a = single_action(-1, 0, 0, nt); // dix=-1 → 範囲外
    let s = states[0].clone();
    let c = super::action_cost_raw(&states, &a, &s, 0, 1, 1, nt as i32, &BeliefModel::default());
    assert_eq!(c, super::MAX_COST);
}

#[test]
fn action_cost_obstacle_neighbor_returns_max() {
    let nt = 1usize;
    let states = vec![
        mk_state(0, 0, 0, true, 0, 0),
        mk_state(1, 0, 0, false, 0, 0), // not free
    ];
    let a = single_action(1, 0, 0, nt);
    let s = states[0].clone();
    let c = super::action_cost_raw(&states, &a, &s, 0, 2, 1, nt as i32, &BeliefModel::default());
    assert_eq!(c, super::MAX_COST);
}

#[test]
fn action_cost_overflow_wraps() {
    // 未到達 free 隣接 (total=MAX_COST) → MAX_COST*PROB_BASE が u64 を折り返す。
    // 期待値: (MAX_COST + PROB_BASE) を PROB_BASE 倍して wrap し >>18。
    let nt = 1usize;
    let penalty = super::PROB_BASE;
    let states = vec![
        mk_state(0, 0, 0, true, super::MAX_COST, penalty),
        mk_state(1, 0, 0, true, super::MAX_COST, penalty),
    ];
    let a = single_action(1, 0, 0, nt);
    let s = states[0].clone();
    let c = super::action_cost_raw(&states, &a, &s, 0, 2, 1, nt as i32, &BeliefModel::default());
    // 手計算: term = (MAX_COST + PROB_BASE) wrapping_mul PROB_BASE; result = term >> 18。
    let term = (super::MAX_COST.wrapping_add(penalty)).wrapping_mul(super::PROB_BASE);
    let expected = term >> super::PROB_BASE_BIT;
    assert_eq!(c, expected);
    // 折り返しにより MAX_COST 未満になることを確認 (固有挙動)。
    assert!(c < super::MAX_COST, "overflow wrap should yield value < MAX_COST, got {c}");
}

#[test]
fn value_iteration_picks_min_and_records_action() {
    // 3x1 マップ θ=0。中央 (idx=1) から action0:dix=+1(隣 total=9), action1:dix=-1(隣 total=4)。
    let nt = 1usize;
    let nx = 3;
    let ny = 1;
    let mut states = vec![
        mk_state(0, 0, 0, true, 4 * super::PROB_BASE, super::PROB_BASE),
        mk_state(1, 0, 0, true, super::MAX_COST, super::PROB_BASE),
        mk_state(2, 0, 0, true, 9 * super::PROB_BASE, super::PROB_BASE),
    ];
    let a0 = single_action(1, 0, 0, nt); // → 右 (total=9)
    let a1 = single_action(-1, 0, 0, nt); // → 左 (total=4)
    let actions = vec![a0, a1];
    let mid = 1usize;
    let d = super::value_iteration_raw(&mut states, &actions, mid, nx, ny, nt as i32, &BeliefModel::default());
    // 左 (4*PB + PB)*PB >>18 = 5*PB。右 = 10*PB。min = 5*PB、action1。
    assert_eq!(states[mid].total_cost, 5 * super::PROB_BASE);
    assert_eq!(states[mid].optimal_action, Some(1));
    // delta = |5*PB - MAX_COST|
    assert_eq!(d, super::MAX_COST - 5 * super::PROB_BASE);
}

#[test]
fn value_iteration_skips_final_and_obstacle() {
    let nt = 1usize;
    let mut s_final = mk_state(0, 0, 0, true, super::MAX_COST, super::PROB_BASE);
    s_final.final_state = true;
    let mut states = vec![s_final];
    let actions: Vec<Action> = vec![single_action(1, 0, 0, nt)];
    let d = super::value_iteration_raw(&mut states, &actions, 0, 1, 1, nt as i32, &BeliefModel::default());
    assert_eq!(d, 0);
    assert_eq!(states[0].total_cost, super::MAX_COST); // 未更新
}

#[test]
fn set_goal_normalizes_theta() {
    let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
    let map = free_grid(3, 3);
    vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
    vi.set_goal(0.0, 0.0, -10);
    assert_eq!(vi.goal_t, 350);
    vi.set_goal(0.0, 0.0, 370);
    assert_eq!(vi.goal_t, 10);
    assert_eq!(vi.status, "calculating");
}

#[test]
fn set_state_values_pins_goal_cell() {
    // goal をグリッド角 (0.5,0.5) に置く。final_state は「セルの両角がゴール半径内」
    // を要求するため、角を共有する 4 セルの遠い角 (距離 √2*0.05≈0.0707m) を包む
    // R=0.08 を使う。margin_theta=360 で全θ許容。
    let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
    let map = free_grid(20, 20); // res=0.05 → 範囲 1.0m
    vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.08, 360);
    vi.set_goal(0.5, 0.5, 0); // セル角 (10,10)=(0.5,0.5)
    // (ix=10,iy=10): 左下角=ゴール(r0=0)、右上角 r1=0.005 < 0.08^2=0.0064 → final。
    let idx = vi.to_index(10, 10, 0) as usize;
    assert!(vi.states[idx].final_state);
    assert_eq!(vi.states[idx].total_cost, 0);
    // 遠方セル (0,0) は距離 ≫ R → final でない。
    let far = vi.to_index(0, 0, 0) as usize;
    assert!(!vi.states[far].final_state);
    assert_eq!(vi.states[far].total_cost, super::MAX_COST);
}

#[test]
fn make_value_function_map_wraps_to_i8() {
    let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
    let map = free_grid(2, 2);
    vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
    // set_goal を呼ばない → 全セル free・total_cost=MAX_COST のまま。
    // cost=MAX_COST/PROB_BASE ≫ threshold(60) かつ free → 250 → i8 にラップして -6。
    let og = vi.make_value_function_map(60, 0.0, 0.0, 0.0);
    assert_eq!(og.width, 2);
    assert_eq!(og.height, 2);
    assert!(og.data.iter().all(|&v| v == (250u8 as i8)));
    assert_eq!(250u8 as i8, -6);
}

#[test]
fn status_transitions() {
    let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
    assert_eq!(vi.status, "init");
    vi.set_calculated();
    assert!(vi.is_calculated());
    vi.set_cancel();
    assert!(vi.end_of_trial());
    vi.set_calculated(); // canceled からは変えない
    assert_eq!(vi.status, "canceled");
}

#[test]
fn policy_writer_marks_unset_as_minus_one() {
    let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
    let map = free_grid(2, 2);
    vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
    vi.set_goal(0.0, 0.0, 0);
    let pol = vi.policy_writer();
    assert_eq!(pol.layers.len(), 60);
    // 未計算なので全 -1。
    assert!(pol.layers[0].iter().all(|&v| v == -1.0));
}

// ═══ belief 次元 (x,y,θ,b) ═══

/// `nb_levels == 1` は**構成的に**従来と同一であること (他の belief パラメータが
/// どんな値でも索引・数式に一切現れない)。これが「既定は bit-exact」の担保。
#[test]
fn belief_nb1_is_bit_identical() {
    use crate::solvers::{solve, U64Solver};
    let mut occ = vec![0i8; 8 * 8];
    for iy in 0..8 {
        occ[(iy * 8 + 5) as usize] = 100;
    }
    occ[5] = 0;

    let mut plain = crate::solvers::test_support::make_vi(8, 8, occ.clone());
    // nb_levels 以外はでたらめ。1 層なら一切効かないこと。
    let junk = super::BeliefModel {
        nb_levels: 1,
        ib_goal: 7,
        motion_gain: 3,
        info_near_m: 1.5,
        info_far_m: 9.0,
        ..Default::default()
    };
    let mut belief = crate::solvers::test_support::make_vi_with(8, 8, occ, junk);
    assert_eq!(plain.states.len(), belief.states.len());
    solve(&mut plain, U64Solver::Frontier2D, 4000);
    solve(&mut belief, U64Solver::Frontier2D, 4000);
    for (i, (a, b)) in plain.states.iter().zip(belief.states.iter()).enumerate() {
        assert_eq!(a.total_cost, b.total_cost, "total_cost @ {i}");
        assert_eq!(a.optimal_action, b.optimal_action, "optimal_action @ {i}");
        assert_eq!(a.final_state, b.final_state, "final_state @ {i}");
    }
}

/// `info` 場は壁までの city-block 距離を near/far で 3 値化したもの。
/// 地図の外周は壁扱いしない。
#[test]
fn belief_info_map_thresholds() {
    let mut data = vec![0i8; 20 * 5];
    for iy in 0..5 {
        data[(iy * 20 + 10) as usize] = 100; // 縦壁 (ix=10)
    }
    let map = OccupancyGrid {
        width: 20,
        height: 5,
        resolution: 0.05,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: Default::default(),
        data,
    };
    let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
    vi.belief = super::BeliefModel {
        nb_levels: 2,
        info_near_m: 0.1,  // 2 セル
        info_far_m: 0.25,  // 5 セル
        ..Default::default()
    };
    vi.set_map_with_occupancy_grid(&map, 4, 0.0, 30.0, 0.2, 180);
    let at = |ix: i32, iy: i32| vi.belief.info[(iy * 20 + ix) as usize];
    assert_eq!(at(10, 2), 2, "壁セルそのもの (d=0)");
    assert_eq!(at(8, 2), 2, "d=2 <= near");
    assert_eq!(at(7, 2), 1, "d=3 は far のみ");
    assert_eq!(at(5, 2), 1, "d=5 <= far");
    assert_eq!(at(4, 2), 0, "d=6 > far");
    assert_eq!(at(0, 0), 0, "地図の外周は壁ではない (d=10)");
}

#[test]
fn belief_next_ib_clamps() {
    // info = [0, 1, 2] を ix=0,1,2 に置いた 3x1 の場。
    let bm = super::BeliefModel {
        nb_levels: 4,
        motion_gain: 1,
        info: vec![0, 1, 2],
        nx: 3,
        ..Default::default()
    };
    assert_eq!(bm.next_ib(0, 0, 0), 1, "情報なし → b 増");
    assert_eq!(bm.next_ib(3, 0, 0), 3, "上端クランプ");
    assert_eq!(bm.next_ib(2, 1, 0), 2, "info=1 は据え置き");
    assert_eq!(bm.next_ib(2, 2, 0), 1, "info=2 → b 減");
    assert_eq!(bm.next_ib(0, 2, 0), 0, "下端クランプ");
    // nb_levels == 1 では info も motion_gain も見ない。
    let one = super::BeliefModel { nb_levels: 1, info: vec![0, 1, 2], nx: 3, ..Default::default() };
    assert_eq!(one.next_ib(5, 0, 0), 0);
}

/// ゴール開放は `ib <= ib_goal` の層だけ。不確かな層ではゴールが終端にならない。
#[test]
fn belief_goal_gating() {
    let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
    vi.belief = super::BeliefModel { nb_levels: 3, ib_goal: 0, ..Default::default() };
    let map = free_grid(20, 20);
    vi.set_map_with_occupancy_grid(&map, 4, 0.2, 30.0, 0.08, 360);
    assert_eq!(vi.states.len(), 20 * 20 * 4 * 3);
    vi.set_goal(0.5, 0.5, 0);
    let goal = vi.to_index4(10, 10, 0, 0);
    assert!(vi.states[goal].final_state, "層 0 のゴールは終端");
    assert_eq!(vi.states[goal].total_cost, 0);
    for ib in 1..3 {
        let i = vi.to_index4(10, 10, 0, ib);
        assert!(!vi.states[i].final_state, "層 {ib} のゴールは終端でない");
        assert_eq!(vi.states[i].total_cost, super::MAX_COST, "層 {ib} は MAX_COST から");
    }
}

#[test]
fn value_function_writer_truncates_substep() {
    // ★本家 valueFunctionWriter は total_cost_/prob_base_ の整数除算で小数を切り捨てる。
    // total_cost = 1.5 step (PROB_BASE + PROB_BASE/2) → 報告値は floor = 1.0。
    let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
    let map = free_grid(2, 2);
    vi.set_map_with_occupancy_grid(&map, 4, 0.2, 30.0, 0.2, 10);
    let idx = vi.to_index(0, 0, 0) as usize;
    vi.states[idx].total_cost = super::PROB_BASE + super::PROB_BASE / 2; // 1.5 step
    let gl = vi.value_function_writer();
    // layer[theta=0] の (iy=0,ix=0) = floor(1.5) = 1.0 (浮動小数除算なら 1.5 になる)。
    assert_eq!(gl.layers[0][0], 1.0);
}
