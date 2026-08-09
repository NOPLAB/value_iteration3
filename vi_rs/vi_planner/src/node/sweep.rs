//! 狭域 → 広域のフィードバック: 共有価値関数の全域掃き (背景スレッド 1 本)。

use std::sync::Arc;
use std::time::{Duration, Instant};

use vi_planner::core::{lock, SweepCursor};

use super::handles::Handles;
use super::msg::ros_grid_from;
use super::params::Params;

/// 全域掃きスレッドを立てる (`global_sweep: false` なら何もしない)。
///
/// 追従 (`observe_scan`) が書いた local_penalty はローカルウィンドウ (±1m) の
/// 中で値を上げるだけで、そこから外へは広がらない。ここで同じ `states` を
/// 全域 Gauss–Seidel で掃き直して初めて、広域の経路が塞がった通路を避ける
/// ようになる (`core::sweep_global` の doc)。
///
/// compact 経路も同じ入口で回る。向こうは全域の `states` を持てないので、
/// 1 呼び出しで sink のタイルを 1 枚だけ起こして掃いて書き戻す
/// (`core::Repair`)。伝播にかかる時間は地図の大きさではなく変化が及ぶ範囲に
/// 比例するので、最初の反応が遅くても掃き終わるまでの実測ログを見ること。
///
/// **`value_function` の再配信もこのスレッドが持つ**。追従中は solve が
/// 走らないので、ここで出さないと全域スライスは solve した瞬間のまま固まり、
/// 伝播が効いていても画面では何も起きない (追従ループが出すのは ±1m の窓だけ
/// で、窓は機体と一緒に動くので離れると固まったままの全域が下から見える)。
///
/// ロックの持ち方が肝。10 Hz の追従ループは同じ Mutex を `try_lock` で取り、
/// 3 tick 続けて取れないとロボットを止める。そこで 1 掃きを走り切らせず、
/// budget ms だけ掃いてロックを手放し、idle ms 待つ形にしてある
/// (既定 20:60 = 1 コアの 25%)。
pub fn spawn(handles: Arc<Handles>, params: &Params) {
    // 早期走り出し (early_start) は「残りは走りながら背景で解き切る」が前提なので、
    // 掃きが立っていないと未確定のまま走り続けることになる。核の PlanConfig 側も
    // 同じ判断で global_sweep を立てている (node::boot)。
    if !params.global_sweep && !params.early_start {
        return;
    }
    let h = handles;
    let budget = Duration::from_millis(params.global_sweep_budget_ms.max(1) as u64);
    let idle = Duration::from_millis(params.global_sweep_idle_ms.max(0) as u64);
    // 密経路の 1 チャンクの粒度。budget の中で経過時間を見る刻みでもあるので、
    // Pi4 (実測 1 掃き 7.9M セルで 8-11 秒 = 約 1M cells/s) で数 ms になる
    // 大きさにしてある。compact 経路はこの値を使わず、1 呼び出し = タイル 1 枚
    // (これも数十 ms で頭打ち)。
    let cells_per_step = params.global_sweep_cells_per_step.max(1) as usize;
    // 伝播が続いている間の報告間隔。**走行中は待ち行列がまず空にならない**
    // ので (壁が窓に入っていれば `set_local_cost` が毎 tick penalty を塗り
    // 直す)、下の「done」の行はほとんど出ない。掃きが動いているかを見る
    // 手掛かりはこの間隔で出る進捗のほうになる。
    //
    // 価値関数の再配信もここに乗せる。追従中は solve が走らないので、
    // ここで出さないと `value_function` は solve した瞬間のまま固まり、
    // **伝播が効いていても画面では何も起きない** (追従ループが出すのは
    // ローカルウィンドウだけ)。全域 1 枚は 19F (scale 2) で 13 万セル、津田沼
    // (scale 5) で 94 万セル読むので、10 Hz の追従ループには置けない。
    // 作るのはロックの中なので、この間隔の tick だけ追従ループが try_lock に
    // 失敗し得る (2 秒に 1 回なら 3 tick 連続にはならず、機体は止まらない)。
    let report_every = Duration::from_secs_f64(params.global_sweep_report_sec.max(0.1));
    std::thread::spawn(move || {
        let mut cur = SweepCursor::default();
        let mut sweep_start: Option<Instant> = None;
        let mut last_report = Instant::now();
        loop {
            // ロックの中で作って、外で配信する (可視化 1 枚は 100 万セル級)。
            let mut grid = None;
            {
                let mut c = lock(&h.core);
                if !c.is_dirty() {
                    // 掃く仕事が無い。ロックはすぐ返して、次に狭域が場を動かす
                    // まで長めに待つ (この確認自体もロックを要るので、間隔を
                    // 詰めると追従ループから取り上げてしまう)。
                    drop(c);
                    std::thread::sleep(idle.max(Duration::from_millis(200)));
                    continue;
                }
                let t0 = Instant::now();
                let started = *sweep_start.get_or_insert(t0);
                let mut done = false;
                loop {
                    if c.sweep_global(&mut cur, cells_per_step).1 {
                        done = true;
                        break;
                    }
                    if t0.elapsed() >= budget {
                        break;
                    }
                }

                if done || last_report.elapsed() >= report_every {
                    last_report = Instant::now();
                    if done {
                        // 伝播 1 回に実際かかった時間 (idle を含む実時間 =
                        // 狭域の判断が広域に届くまでの遅れそのもの)。budget/idle を
                        // 実測から詰められるよう必ず出す。`still_dirty=false` なら
                        // 新しい不動点に達したので、次の狭域の書き込みまで止まる。
                        //
                        // compact は 1 回の伝播で掃く量が地図の大きさで決まらない
                        // (変化が及んだ範囲で決まる) ので、経過時間だけでは速いのか
                        // 仕事が少なかったのか読めない。タイル数を必ず添える。
                        let work = match c.sweep_tiles() {
                            Some(n) => format!(", {n} tiles"),
                            None => String::new(),
                        };
                        eprintln!(
                            "vi_planner: global sweep done in {:.1}s{} (still_dirty={})",
                            started.elapsed().as_secs_f64(),
                            work,
                            c.is_dirty()
                        );
                        sweep_start = None;
                    } else if let Some((visits, queued)) = c.repair_progress() {
                        eprintln!(
                            "vi_planner: tile repair running for {:.1}s \
                             ({visits} visits, {queued} tiles queued)",
                            started.elapsed().as_secs_f64()
                        );
                    }
                    // `interval: 0` は「solve 完了時のみ」なので、途中の
                    // 再配信はしない (伝播 1 回の完了は完了として出す)。
                    if let Some(v) = h.viz.as_ref() {
                        if done || !v.interval.is_zero() {
                            grid = c.value_grid(v.threshold_steps);
                        }
                    }
                }
            }
            if let (Some(v), Some(g)) = (h.viz.as_ref(), grid) {
                let _ = v.vf_pub.publish(ros_grid_from(&g, &v.frame_id, v.stamp()));
            }
            std::thread::sleep(idle);
        }
    });
}
