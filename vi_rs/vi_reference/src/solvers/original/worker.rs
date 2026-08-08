//! 本家 `valueIterationWorker` の実行経路 (単スレッド/マルチスレッド)。
//! `ValueIterator` のメソッドとして定義する — モデル (`value_iterator.rs`) が持つのは
//! 状態と per-cell Bellman 更新までで、それをどう走査するか (= 本家ソルバ) はここ。

use crate::params::PROB_BASE_BIT;
use crate::state::State;
use crate::value_iterator::{value_iteration_raw, ValueIterator};

use super::sweep_status::SweepWorkerStatus;

/// `*mut State` をスレッド間共有するためのラッパ。
/// SAFETY: 本家の non-atomic 共有 `states_` のデータ競合を**忠実再現**するための
/// 意図的な共有可変。`thread_num>1` は本家同様に非決定的 (技術的 UB、x86 で動く)。
#[derive(Clone, Copy)]
struct StatesPtr(*mut State);
unsafe impl Send for StatesPtr {}
unsafe impl Sync for StatesPtr {}

impl ValueIterator {
    /// 本家 `valueIterationWorker`。単スレッド経路 (決定的・テスト基準)。
    /// `times` 回スイープ。`status` が canceled/goal なら中断。
    pub fn value_iteration_worker(&mut self, times: i32, id: i32) {
        self.thread_status.insert(id, SweepWorkerStatus::default());
        let order_idx = (id as usize) % self.sweep_orders.len();

        for j in 0..times {
            if let Some(st) = self.thread_status.get_mut(&id) {
                st.sweep_step = j + 1;
            }
            let mut max_delta: u64 = 0;
            let order_len = self.sweep_orders[order_idx].len();
            for k in 0..order_len {
                let i = self.sweep_orders[order_idx][k] as usize;
                let d = self.value_iteration_at(i);
                if d > max_delta {
                    max_delta = d;
                }
            }
            if let Some(st) = self.thread_status.get_mut(&id) {
                st.delta = (max_delta >> PROB_BASE_BIT) as f64; // ★二重シフト (報告用)
            }
            if self.status == "canceled" || self.status == "goal" {
                break;
            }
        }
        if let Some(st) = self.thread_status.get_mut(&id) {
            st.finished = true;
        }
    }

    /// 本家 `finished`。thread 0..thread_num の状態を集約。
    /// std::map operator[] の既定挿入を `entry().or_default()` で再現。
    pub fn finished(&mut self) -> (Vec<u32>, Vec<f64>, bool) {
        let n = self.thread_num as usize;
        let mut sweep_times = vec![0u32; n];
        let mut deltas = vec![0f64; n];
        let mut finish = true;
        for t in 0..self.thread_num {
            let st = self.thread_status.entry(t).or_default();
            sweep_times[t as usize] = st.sweep_step as u32;
            deltas[t as usize] = st.delta;
            finish &= st.finished;
        }
        (sweep_times, deltas, finish)
    }

    /// 価値反復を実行するエントリ。`thread_num<=1` は単スレッド (決定的)。
    /// `thread_num>1` は Task 14 のマルチスレッド経路を使う。
    pub fn run_value_iteration(&mut self, times: i32) {
        if self.thread_num <= 1 {
            self.value_iteration_worker(times, 0);
        } else {
            self.run_value_iteration_multithread(times);
        }
    }

    /// 本家 `valueIterationWorker` をスレッドごとに spawn したマルチスレッド経路。
    /// 共有 `states` を生ポインタ経由で non-atomic 並行更新する (本家のデータ競合を再現)。
    /// `status`/`thread_status` は安全側で扱う (バッチ実行では status は不変)。
    fn run_value_iteration_multithread(&mut self, times: i32) {
        self.thread_status.clear();

        let n_states = self.states.len();
        let ptr = StatesPtr(self.states.as_mut_ptr());
        let cell_num_x = self.cell_num_x;
        let cell_num_y = self.cell_num_y;
        let cell_num_t = self.cell_num_t;
        let thread_num = self.thread_num;
        let actions = &self.actions;
        let sweep_orders = &self.sweep_orders;
        // バッチ実行中は status は不変なので break 条件を bool (Copy) で先に確定し、
        // 各スレッドクロージャへ move キャプチャする (String を多重 move できないため)。
        let stop = self.status == "canceled" || self.status == "goal";

        let results: Vec<(i32, SweepWorkerStatus)> = std::thread::scope(|scope| {
            let handles: Vec<_> = (0..thread_num)
                .map(|id| {
                    scope.spawn(move || {
                        // edition 2021 disjoint capture: force capture of StatesPtr wrapper,
                        // not ptr.0 field (*mut State which is !Send).
                        let ptr = ptr;
                        // SAFETY: 全スレッドが同一バッファを共有。本家のデータ競合を忠実再現。
                        let states: &mut [State] =
                            unsafe { std::slice::from_raw_parts_mut(ptr.0, n_states) };
                        let mut st = SweepWorkerStatus::default();
                        let order = &sweep_orders[(id as usize) % sweep_orders.len()];
                        for j in 0..times {
                            st.sweep_step = j + 1;
                            let mut max_delta: u64 = 0;
                            for &si in order.iter() {
                                let d = value_iteration_raw(
                                    states,
                                    actions,
                                    si as usize,
                                    cell_num_x,
                                    cell_num_y,
                                    cell_num_t,
                                );
                                if d > max_delta {
                                    max_delta = d;
                                }
                            }
                            st.delta = (max_delta >> PROB_BASE_BIT) as f64;
                            if stop {
                                break;
                            }
                        }
                        st.finished = true;
                        (id, st)
                    })
                })
                .collect();
            handles.into_iter().map(|h| h.join().unwrap()).collect()
        });

        for (id, st) in results {
            self.thread_status.insert(id, st);
        }
    }
}
