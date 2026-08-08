//! 本家ワーカー実行経路 ([`super::worker`]) のテスト。モデル側
//! (`value_iterator_tests.rs`) から移設。

use crate::action::Action;
use crate::msg::OccupancyGrid;
use crate::params::MAX_COST;
use crate::value_iterator::ValueIterator;

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
fn single_thread_converges_on_small_free_map() {
    // 5x5 free マップ、goal を中央セルに。十分スイープして goal 隣接が確定する。
    let mut vi = ValueIterator::new(crate::solvers::test_support::actions(), 1);
    let map = free_grid(5, 5);
    vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
    vi.set_goal(0.1, 0.1, 0); // セル (2,2) 付近

    vi.run_value_iteration(300);

    // 何らかの非 goal セルが MAX_COST 未満 (= 到達可能) になっていること。
    let reachable = vi.states.iter().any(|s| !s.final_state && s.total_cost < MAX_COST);
    assert!(reachable, "value should propagate from goal");

    // 2 回目の実行で値が変わらない (収束済み) ことを idempotent で確認。
    let before: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
    vi.run_value_iteration(50);
    let after: Vec<u64> = vi.states.iter().map(|s| s.total_cost).collect();
    assert_eq!(before, after, "converged values must be stable");
}

#[test]
fn finished_aggregates_thread_status() {
    let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 1);
    let map = free_grid(3, 3);
    vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
    vi.set_goal(0.0, 0.0, 0);
    vi.value_iteration_worker(3, 0);
    let (sweeps, _deltas, finish) = vi.finished();
    assert_eq!(sweeps.len(), 1);
    assert_eq!(sweeps[0], 3);
    assert!(finish);
}

#[test]
fn multithread_converges_close_to_single_thread() {
    // 同一マップ・ゴールで、マルチスレッド (データ競合あり・非決定的) が
    // 単スレッドと同程度に値を伝播し、近い解へ収束することを確認 (bit 一致は要求しない)。
    let build = |threads: i32| {
        let mut vi = ValueIterator::new(
            vec![
                Action::new("forward", 0.3, 0.0, 0),
                Action::new("back", -0.2, 0.0, 1),
                Action::new("left", 0.0, 20.0, 4),
            ],
            threads,
        );
        let map = free_grid(6, 6);
        vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
        vi.set_goal(0.1, 0.1, 0);
        vi
    };

    let mut single = build(1);
    single.run_value_iteration(500);

    let mut multi = build(4);
    multi.run_value_iteration(500);

    // thread_num>1 は本家同様データ競合で非決定的 → bit 一致は要求しない。
    // 「マルチスレッドも単スレッドと同程度に値を伝播し、折り返し garbage を残さない」ことを確認。
    let finite = |vi: &ValueIterator| {
        vi.states.iter().filter(|s| s.total_cost < MAX_COST).count()
    };
    let max_finite = |vi: &ValueIterator| {
        vi.states
            .iter()
            .map(|s| s.total_cost)
            .filter(|&c| c < MAX_COST)
            .max()
            .unwrap_or(0)
    };
    let sf = finite(&single);
    let mf = finite(&multi);
    assert!(sf > 0, "single-thread should propagate values");
    assert!(
        mf >= sf * 9 / 10,
        "multi-thread coverage should be close to single (single={sf}, multi={mf})"
    );
    assert!(
        max_finite(&multi) <= max_finite(&single) * 2,
        "multi-thread must not leave overflow-wrapped garbage values"
    );
}

#[test]
fn multithread_finished_reports_all_threads() {
    let mut vi = ValueIterator::new(vec![Action::new("f", 0.3, 0.0, 0)], 3);
    let map = free_grid(4, 4);
    vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.2, 10);
    vi.set_goal(0.0, 0.0, 0);
    vi.run_value_iteration(5);
    let (sweeps, _d, finish) = vi.finished();
    assert_eq!(sweeps.len(), 3);
    assert!(finish);
    assert!(sweeps.iter().all(|&s| s == 5));
}
