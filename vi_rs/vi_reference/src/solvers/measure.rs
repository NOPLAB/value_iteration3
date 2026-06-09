//! Step 0 計測 (オラクル不変・テスト専用)。frontier2d の律速を切り分けるための instrumented 実行。
//! `frontier2d.rs` のロジックは複製して計測子を仕込む (本家忠実移植のコアは変更しない)。
//!
//! 実行 (host /tmp ワークアラウンド):
//!   cd /tmp && CARGO_TARGET_DIR=/tmp/vi_meas \
//!     cargo test --release --manifest-path <repo>/vi_rs/Cargo.toml -p vi_reference \
//!     measure::measure_frontier2d_hotpath -- --ignored --nocapture

use std::time::{Duration, Instant};

use super::{displacement, seed_frontier_2d, Bitboard2D};
use crate::action::Action;
use crate::msg::{OccupancyGrid, Quaternion};
use crate::state::State;
use crate::value_iterator::{value_iteration_raw, ValueIterator};

fn house_pgm_path() -> Option<String> {
    if let Ok(p) = std::env::var("VI_HOUSE_PGM") {
        return Some(p);
    }
    for c in [
        "../value_iteration/maps/house.pgm",
        "/home/nop/dev/mywork/value_iteration/maps/house.pgm",
    ] {
        if std::path::Path::new(c).exists() {
            return Some(c.to_string());
        }
    }
    None
}

/// P5 PGM を読む (コメント行スキップ)。返り値 (w, h, pixels row-major)。
fn load_pgm(path: &str) -> (i32, i32, Vec<u8>) {
    let bytes = std::fs::read(path).expect("read pgm");
    // ヘッダトークンをホワイトスペース/コメント無視で 4 つ読む: P5, w, h, maxval。
    let mut pos = 0usize;
    let mut toks: Vec<String> = Vec::new();
    while toks.len() < 4 {
        // skip whitespace
        while pos < bytes.len() && bytes[pos].is_ascii_whitespace() {
            pos += 1;
        }
        if pos < bytes.len() && bytes[pos] == b'#' {
            while pos < bytes.len() && bytes[pos] != b'\n' {
                pos += 1;
            }
            continue;
        }
        let start = pos;
        while pos < bytes.len() && !bytes[pos].is_ascii_whitespace() {
            pos += 1;
        }
        toks.push(String::from_utf8_lossy(&bytes[start..pos]).to_string());
    }
    assert_eq!(toks[0], "P5");
    let w: i32 = toks[1].parse().unwrap();
    let h: i32 = toks[2].parse().unwrap();
    // maxval の後の単一ホワイトスペース 1 つを飛ばして binary 開始。
    pos += 1;
    let pixels = bytes[pos..pos + (w * h) as usize].to_vec();
    (w, h, pixels)
}

/// u64_bench.py の to_occupancy と同一 (negate=0, free_thresh=0.196, occupied_thresh=0.65, flipud)。
/// 返り値: ValueIterator が要求する i8 occupancy (row-major, h*w)、free=0 / obstacle=100 / unknown=-1。
fn to_occupancy(w: i32, h: i32, pgm: &[u8]) -> Vec<i8> {
    let (free_thresh, occ_thresh) = (0.196f64, 0.65f64);
    let mut occ = vec![-1i8; (w * h) as usize];
    for y in 0..h {
        for x in 0..w {
            let p = pgm[(y * w + x) as usize] as f64;
            let occ_prob = (255.0 - p) / 255.0;
            let v: i8 = if occ_prob < free_thresh {
                0
            } else if occ_prob > occ_thresh {
                100
            } else {
                -1
            };
            // flipud: 行 y を (h-1-y) へ。
            occ[((h - 1 - y) * w + x) as usize] = v;
        }
    }
    occ
}

fn default_actions() -> Vec<Action> {
    vec![
        Action::new("forward", 0.3, 0.0, 0),
        Action::new("back", -0.2, 0.0, 1),
        Action::new("right", 0.0, -20.0, 2),
        Action::new("rightfw", 0.2, -20.0, 3),
        Action::new("left", 0.0, 20.0, 4),
        Action::new("leftfw", 0.2, 20.0, 5),
    ]
}

fn build_house_vi() -> Option<ValueIterator> {
    let path = house_pgm_path()?;
    let (w, h, pgm) = load_pgm(&path);
    let occ = to_occupancy(w, h, &pgm);
    let map = OccupancyGrid {
        width: w,
        height: h,
        resolution: 0.05,
        origin_x: -10.0,
        origin_y: -10.0,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: occ,
    };
    let mut vi = ValueIterator::new(default_actions(), 1);
    vi.set_map_with_occupancy_grid(&map, 60, 0.2, 30.0, 0.3, 15);
    vi.set_goal(-0.425, -0.425, 0);
    Some(vi)
}

#[test]
#[ignore = "計測専用 (要 house.pgm)。--ignored --nocapture で実行"]
fn measure_frontier2d_hotpath() {
    let Some(mut vi) = build_house_vi() else {
        eprintln!("[measure] house.pgm 未検出 (VI_HOUSE_PGM で指定可)。スキップ。");
        return;
    };
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);

    // ── 静的指標: State サイズ / 配列総量 / θ別バケット数 (内ループ乗数) ──
    let state_bytes = std::mem::size_of::<State>();
    let n_states = vi.states.len();
    eprintln!("--- 静的指標 ---");
    eprintln!("grid = {nx}x{ny}x{nt} = {n_states} states");
    eprintln!(
        "sizeof(State) = {state_bytes} B, states 配列 = {:.1} MB",
        (n_states * state_bytes) as f64 / 1e6
    );
    // B[it] = sum_a len(state_transitions[a][it]) : θ=it のセル 1 更新あたりの隣接バケット総数。
    let mut b_per_theta = vec![0u64; nt as usize];
    for a in &vi.actions {
        for it in 0..nt as usize {
            b_per_theta[it] += a.state_transitions[it].len() as u64;
        }
    }
    let b_total: u64 = b_per_theta.iter().sum();
    let b_avg = b_total as f64 / nt as f64;
    let b_max = *b_per_theta.iter().max().unwrap();
    let b_min = *b_per_theta.iter().min().unwrap();
    eprintln!(
        "内ループ乗数 B (6アクション合計バケット/セル更新): avg={b_avg:.1}, min={b_min}, max={b_max}"
    );
    for a in &vi.actions {
        let tot: usize = (0..nt as usize).map(|it| a.state_transitions[it].len()).sum();
        eprintln!("  action '{}': avg buckets/θ = {:.1}", a.name, tot as f64 / nt as f64);
    }

    // ── instrumented frontier2d (frontier2d_solve のロジック複製 + 計測子) ──
    let (mx, my, _mt) = displacement(&vi);
    let (dx, dy) = (mx as u32, my as u32);
    let mut frontier = seed_frontier_2d(&vi);
    let mut updates: u64 = 0;
    let mut iters: u32 = 0;
    let mut vir_calls: u64 = 0; // value_iteration_raw 呼び出し回数
    let mut potential_bucket_visits: u64 = 0; // Σ B[it] (隣接ロード上限 = メモリ流量の主項)
    let mut dilate_time = Duration::ZERO;
    let mut compute_time = Duration::ZERO;
    let max_iter = 2000u32;

    let t_total = Instant::now();
    while frontier.popcount() > 0 && iters < max_iter {
        iters += 1;
        let t_d = Instant::now();
        let candidates = frontier.dilate(dx, dy);
        dilate_time += t_d.elapsed();

        let t_c = Instant::now();
        let mut new_frontier = Bitboard2D::new(nx as u32, ny as u32);
        for (ix, iy) in candidates.enumerate() {
            let mut changed = false;
            for it in 0..nt {
                let idx = vi.to_index(ix as i32, iy as i32, it) as usize;
                let before = vi.states[idx].total_cost;
                value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt);
                vir_calls += 1;
                potential_bucket_visits += b_per_theta[it as usize];
                if vi.states[idx].total_cost < before {
                    updates += 1;
                    changed = true;
                }
            }
            if changed {
                new_frontier.set(ix, iy);
            }
        }
        compute_time += t_c.elapsed();
        frontier = new_frontier;
    }
    let total = t_total.elapsed();

    let bytes_touched = potential_bucket_visits * state_bytes as u64;
    eprintln!("--- 実行計測 (instrumented frontier2d) ---");
    eprintln!("iters={iters}, updates(減少した更新)={updates}, converged={}", frontier.popcount() == 0);
    eprintln!("value_iteration_raw 呼び出し = {vir_calls} ({:.1}M)", vir_calls as f64 / 1e6);
    eprintln!(
        "潜在バケット訪問 ΣB[it] = {potential_bucket_visits} ({:.0}M) = 隣接 State ロード上限",
        potential_bucket_visits as f64 / 1e6
    );
    eprintln!(
        "推定メモリ流量 (バケット訪問 × sizeof State) ≈ {:.1} GB",
        bytes_touched as f64 / 1e9
    );
    eprintln!(
        "時間配分: total={:.3}s | dilate={:.3}s ({:.1}%) | compute={:.3}s ({:.1}%)",
        total.as_secs_f64(),
        dilate_time.as_secs_f64(),
        100.0 * dilate_time.as_secs_f64() / total.as_secs_f64(),
        compute_time.as_secs_f64(),
        100.0 * compute_time.as_secs_f64() / total.as_secs_f64(),
    );
    eprintln!(
        "compute あたり: {:.1} ns / VIR呼び出し, {:.2} ns / バケット訪問",
        compute_time.as_nanos() as f64 / vir_calls as f64,
        compute_time.as_nanos() as f64 / potential_bucket_visits as f64,
    );
    eprintln!(
        "実効帯域 (compute 時間で割る) ≈ {:.1} GB/s",
        bytes_touched as f64 / compute_time.as_secs_f64() / 1e9
    );
}
