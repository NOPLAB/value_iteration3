//! 直径レジーム測定（ホスト完結・Docker/ROS 非依存）。合成 free ストリップで
//! reference / frontier2d / prio_ls / prio_lc の elapsed・更新数・repops を比較し、
//! markdown 表で出力する。設計 §5.2。
//!
//!   cargo run --release -p vi_reference --bin vi_prio_measure

// 正典 6 アクションほか (vi_ref_bench / vi_u64_bench と同じ include)。
// この bin は default_actions しか使わないので未使用ヘルパの警告は落とす。
#[allow(dead_code)]
#[path = "bench_common/mod.rs"]
mod bench_common;

use std::time::Instant;

use bench_common::default_actions;
use vi_reference::solvers::priority::priority_solve;
use vi_reference::solvers::{solve, U64Solver, REACH_THRESH as REACH};
use vi_reference::{OccupancyGrid, Quaternion, ValueIterator};

fn build(w: i32, h: i32) -> ValueIterator {
    let mut vi = ValueIterator::new(default_actions(), 1);
    let map = OccupancyGrid {
        width: w,
        height: h,
        resolution: 0.05,
        origin_x: 0.0,
        origin_y: 0.0,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: vec![0i8; (w * h) as usize],
    };
    vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.3, 15);
    // ゴールは左端中央付近に置き、横長マップで直径 ≈ 幅/ステップ を最大化。
    let gy = h as f64 * 0.5 * 0.05;
    vi.set_goal(0.10, gy, 0);
    vi
}

fn reach_count(vi: &ValueIterator) -> u64 {
    vi.states.iter().filter(|s| s.total_cost < REACH).count() as u64
}

fn main() {
    let sizes = [(512, 64), (1024, 64), (2048, 64)];
    println!("| map | solver | elapsed[s] | pops/sweeps | updates | upd/reach | repops | reach |");
    println!("|---|---|---|---|---|---|---|---|");
    for (w, h) in sizes {
        // 到達セルの分母は厳密な prio_lc から取る（unreachable は MAX_COST 据置）。
        let mut vi_lc = build(w, h);
        let t = Instant::now();
        let lc = priority_solve(&mut vi_lc, 3000, false);
        let e_lc = t.elapsed().as_secs_f64();
        let rc = reach_count(&vi_lc).max(1);

        // reference（全走査）。
        let mut vi = build(w, h);
        let t = Instant::now();
        let st = solve(&mut vi, U64Solver::Reference, 3000);
        let e = t.elapsed().as_secs_f64();
        println!("| {w}x{h} | reference | {e:.3} | {} | - | - | - | {rc} |", st.iters);

        // frontier2d（活性集合）。
        let mut vi = build(w, h);
        let t = Instant::now();
        let st = solve(&mut vi, U64Solver::Frontier2D, 3000);
        let e = t.elapsed().as_secs_f64();
        println!(
            "| {w}x{h} | frontier2d | {e:.3} | {} | {} | {:.2} | - | {rc} |",
            st.iters,
            st.updates,
            st.updates as f64 / rc as f64
        );

        // prio_ls（近似・settle-once）。
        let mut vi = build(w, h);
        let t = Instant::now();
        let st = priority_solve(&mut vi, 3000, true);
        let e = t.elapsed().as_secs_f64();
        println!(
            "| {w}x{h} | prio_ls | {e:.3} | {} | {} | {:.2} | {} | {rc} |",
            st.iters,
            st.updates,
            st.updates as f64 / rc as f64,
            st.repops
        );

        // prio_lc（厳密・label-correcting）。
        println!(
            "| {w}x{h} | prio_lc | {e_lc:.3} | {} | {} | {:.2} | {} | {rc} |",
            lc.iters,
            lc.updates,
            lc.updates as f64 / rc as f64,
            lc.repops
        );
    }
}
