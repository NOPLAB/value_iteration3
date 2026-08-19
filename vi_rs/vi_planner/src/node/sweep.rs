//! 狭域 → 広域のフィードバック: 共有価値関数の全域伝播 (背景スレッド 1 本)。

use std::sync::Arc;
use std::time::{Duration, Instant};

use vi_planner::core::lock;

use super::handles::Handles;
use super::msg::ros_grid_from;
use super::params::Params;

/// 1 回のロック取得で掃く時間。追従ループの 1 tick (100 ms) より十分短いこと
/// — 手放す間隔がこれで決まる。CPU の取り分は `global_sweep_duty` [%] で、
/// これを固定したまま待ち時間のほうを伸縮させる。
const BUDGET: Duration = Duration::from_millis(20);

/// 密経路の 1 チャンクの粒度 [状態]。[`BUDGET`] の中で経過時間を見る刻みでも
/// あるので、Pi4 (実測 約 1M states/s) で数 ms になる大きさ。候補セル 1 つあたり
/// θ 層ぶんの更新が走るので、セル数ではこの 1/60。compact 経路はこの値を使わない
/// (1 呼び出し = タイル 1 枚)。
const CELLS_PER_STEP: usize = 5_000;

/// 伝播が続いている間の報告間隔。**走行中は待ち行列がまず空にならない**ので
/// (壁が窓に入っていれば `set_local_cost` が毎 tick penalty を塗り直す)、
/// 「done」の行はほとんど出ない。掃きが動いているかを見る手掛かりはこの間隔で
/// 出る進捗のほうになる。
///
/// 価値関数の再配信もこの間隔に乗る。追従中は solve が走らないので、ここで
/// 出さないと `value_function` は solve した瞬間のまま固まり、**伝播が効いて
/// いても画面では何も起きない** (追従ループが出すのはローカルウィンドウだけ)。
/// 全域 1 枚は 19F (scale 2) で 13 万セル、津田沼 (scale 5) で 94 万セル読む。
/// 作るのはロックの中なので、この間隔の tick だけ追従ループが try_lock に
/// 失敗し得る (2 秒に 1 回なら 3 tick 連続にはならず、機体は止まらない)。
///
/// 注意: `value_publish_interval_ms: 0` (= 「落ち着いたときだけ配信」) では
/// この間隔の再配信をせず `done` のときだけ出す。`done` は伝播が不動点に達した
/// ときなので、壁沿いを走り続けるあいだ全域スライスは更新されない。走行中も
/// 動かしたければ既定 (500 ms) のままにすること。
const REPORT_EVERY: Duration = Duration::from_secs(2);

/// 全域伝播スレッドを立てる (`global_sweep: false` なら何もしない)。
///
/// 追従 (`observe_scan`) が書いた local_penalty はローカルウィンドウ (±1m) の
/// 中で値を上げるだけで、そこから外へは広がらない。ここで同じ `states` を
/// 掃き直して初めて、広域の経路が塞がった通路を避けるようになる
/// (`core::sweep_global` の doc)。
///
/// **どちらの経路も掃くのは「変化が届く範囲」だけで、地図の広さには比例しない。**
/// 密は値が動いたセルを遷移の届く距離だけ膨張させた能動集合、compact は sink の
/// タイルを 1 枚ずつ (`core::Repair`)。最初の反応が遅くても、掃き終わるまでの
/// 実測ログを見ること。
///
/// **`value_function` の再配信もこのスレッドが持つ**。追従中は solve が
/// 走らないので、ここで出さないと全域スライスは solve した瞬間のまま固まり、
/// 伝播が効いていても画面では何も起きない (追従ループが出すのは ±1m の窓だけ
/// で、窓は機体と一緒に動くので離れると固まったままの全域が下から見える)。
///
/// ロックの持ち方が肝。10 Hz の追従ループは同じ Mutex を `try_lock` で取り、
/// 3 tick 続けて取れないとロボットを止める。そこで伝播を走り切らせず、
/// [`BUDGET`] だけ掃いてロックを手放し、残りは待つ形にしてある
/// (割合は `global_sweep_duty` [%]、既定 25 = 1 コアの 1/4)。
pub fn spawn(handles: Arc<Handles>, params: &Params) {
    // 早期走り出し (early_start) は「残りは走りながら背景で解き切る」が前提なので、
    // 掃きが立っていないと未確定のまま走り続けることになる。核の PlanConfig 側も
    // 同じ判断で global_sweep を立てている (node::boot)。
    if !params.global_sweep && !params.early_start {
        return;
    }
    let h = handles;
    // 1 回のロック取得で掃く時間と、そのあと手放して待つ時間。比 (duty) が
    // そのまま CPU の取り分になる。100% で待ちなし。
    let duty = params.global_sweep_duty.clamp(1, 100) as f64 / 100.0;
    let idle = BUDGET.mul_f64((1.0 - duty) / duty);
    std::thread::spawn(move || {
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
                    if c.sweep_global(CELLS_PER_STEP).1 {
                        done = true;
                        break;
                    }
                    if t0.elapsed() >= BUDGET {
                        break;
                    }
                }

                if done || last_report.elapsed() >= REPORT_EVERY {
                    last_report = Instant::now();
                    if done {
                        // 伝播 1 回に実際かかった時間 (idle を含む実時間 =
                        // 狭域の判断が広域に届くまでの遅れそのもの)。
                        // `global_sweep_duty` を実測から詰められるよう必ず出す。
                        // ここに来た = 不動点に達した = 次に狭域が場を動かすまで止まる。
                        //
                        // 1 回の伝播で掃く量は地図の大きさで決まらない (変化が及んだ
                        // 範囲で決まる) ので、経過時間だけでは速いのか仕事が少な
                        // かったのか読めない。処理量を必ず添える。
                        let (work, unit) = c.sweep_work();
                        eprintln!(
                            "vi_planner: global propagation settled in {:.1}s ({work} {unit})",
                            started.elapsed().as_secs_f64(),
                        );
                        sweep_start = None;
                    } else if let Some((done_n, queued)) = c.sweep_progress() {
                        let (_, unit) = c.sweep_work();
                        eprintln!(
                            "vi_planner: global propagation running for {:.1}s \
                             ({done_n} {unit} done, {queued} queued)",
                            started.elapsed().as_secs_f64()
                        );
                    }
                    // `interval: 0` は「落ち着いたときのみ」なので、途中の
                    // 再配信はしない (伝播の完了は完了として出す)。
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
