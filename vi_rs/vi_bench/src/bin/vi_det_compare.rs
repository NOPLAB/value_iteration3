//! `vi_det_compare` — 確率的遷移 (本家: セル内 64×64×64 サブサンプルで遷移先を按分) と
//! 決定論的遷移 (1 遷移, prob = PROB_BASE) で解いた価値関数・方策の差を測る。
//!
//! 決定論モデルは 2 種:
//! - `center`: セル中心から 1 サンプルで遷移先を決める
//! - `mode`:   確率モデルの最頻遷移先を 1 本に潰す
//!
//! 出力: 到達可能状態数、両方で到達可能な状態の値差、方策不一致率、
//! ランダム始点からの貪欲ロールアウトの歩数と壁との最小距離。

use std::path::PathBuf;

use clap::Parser;

use vi_bench::params::{canonical_actions, N_THETA};
use vi_bench::pgm;
use vi_lib::params::{MAX_COST, PROB_BASE, PROB_BASE_BIT};
use vi_lib::solvers::{solve, U64Solver, REACH_THRESH as REACH};
use vi_lib::{Action, OccupancyGrid, Quaternion, StateTransition, ValueIterator};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    map: Option<PathBuf>,
    #[arg(long, default_value_t = 1)]
    scale: usize,
    /// `x,y;x,y;...` (world m)。省略時は地図を 4 象限に分けて各象限の最大連結成分内に自動配置。
    #[arg(long)]
    goals: Option<String>,
    #[arg(long, default_value = "frontier2d_par")]
    solver: String,
    /// 結果 JSON (指標 + θ平均した方策不一致マップ, `--grid-step` で間引き)。
    #[arg(long)]
    out: Option<PathBuf>,
    #[arg(long, default_value_t = 4)]
    grid_step: usize,
    /// 図用ダンプ先: θ=0 の価値/方策スライス (PGM)、θ平均不一致率、ロールアウト経路 (JSON)。
    #[arg(long)]
    dump_dir: Option<PathBuf>,
    /// 比較モデル (カンマ区切り)。`center|mode|stochastic` に `_g` (距離勾配ペナルティ) と
    /// `_p<N>` (safety_penalty 上書き, 秒) を後置。例: `center,center_g,stochastic_g,stochastic_p1000`。
    /// `graded` は `center_g` の別名。
    #[arg(long, default_value = "center,graded")]
    models: String,
    /// graded: 余白の外側で safety_penalty を 0 まで線形に落とす距離 [m]。
    #[arg(long, default_value_t = 0.5)]
    ramp_m: f64,
    /// 実行モデル: `continuous` = 連続姿勢で行動を厳密に適用しセルを読む (実機に近い)、
    /// `modal` = 確率モデルの最頻遷移で離散実行。
    #[arg(long, default_value = "continuous")]
    exec: String,
    /// 経路図に描く始点数。
    #[arg(long, default_value_t = 12)]
    path_starts: usize,
    #[arg(long, default_value_t = 0.5)]
    goal_radius_m: f64,
    #[arg(long, default_value_t = 0.0)]
    safety_radius_m: f64,
    #[arg(long, default_value_t = 100000.0)]
    safety_penalty: f64,
    #[arg(long, default_value_t = 200)]
    rollouts: usize,
    #[arg(long, default_value_t = 1.0)]
    action_scale: f64,
}

fn scaled_actions(scale: f64) -> Vec<Action> {
    canonical_actions()
        .into_iter()
        .enumerate()
        .map(|(i, a)| Action::new(&a.name, a.delta_fw * scale, a.delta_rot, i as i32))
        .collect()
}

/// 本家 `cellDelta` (floor, 負側は -ix-1)。
fn cell_delta(x: f64, y: f64, t: f64, r: f64, tr: f64) -> (i32, i32, i32) {
    let f = |v: f64| {
        let i = (v.abs() / r).floor() as i32;
        if v < 0.0 { -i - 1 } else { i }
    };
    (f(x), f(y), (t / tr).floor() as i32)
}

/// セル中心 1 サンプルの決定論遷移に差し替える。
fn make_center(vi: &mut ValueIterator) {
    let (r, tr) = (vi.xy_resolution, vi.t_resolution);
    for a in vi.actions.iter_mut() {
        for (it, list) in a.state_transitions.iter_mut().enumerate() {
            let t = (it as f64 + 0.5) * tr;
            let ang = t.to_radians();
            let (x, y) = (0.5 * r + a.delta_fw * ang.cos(), 0.5 * r + a.delta_fw * ang.sin());
            let mut tt = t + a.delta_rot;
            while tt < 0.0 {
                tt += 360.0;
            }
            let (dx, dy, dt) = cell_delta(x, y, tt, r, tr);
            *list = vec![StateTransition::new(dx, dy, dt, PROB_BASE as i32)];
        }
    }
}

/// 最頻遷移先 1 本に潰す。
fn make_mode(vi: &mut ValueIterator) {
    for a in vi.actions.iter_mut() {
        for list in a.state_transitions.iter_mut() {
            let m = list.iter().max_by_key(|s| s.prob).unwrap();
            *list = vec![StateTransition::new(m.dix, m.diy, m.dit, PROB_BASE as i32)];
        }
    }
}

/// 余白ペナルティを距離勾配にする: L∞ 距離 d が margin 以内は本家どおり、
/// margin+ramp まで線形に 0 へ。free セルだけ書き換える (goal も含む)。
fn make_graded(vi: &mut ValueIterator, dist: &[i32], margin: i32, ramp: f64, penalty: f64) {
    let nx = vi.cell_num_x;
    for s in vi.states.iter_mut() {
        if !s.free {
            continue;
        }
        let d = dist[(s.iy * nx + s.ix) as usize] as f64;
        let f = if d <= margin as f64 { 1.0 } else { (1.0 - (d - margin as f64) / ramp).max(0.0) };
        s.penalty = PROB_BASE + (penalty * f * PROB_BASE as f64) as u64;
    }
}

/// 2D L∞ 障害物距離 (セル)。BFS。
fn obstacle_dist(free: &[bool], nx: i32, ny: i32) -> Vec<i32> {
    let mut d = vec![i32::MAX; (nx * ny) as usize];
    let mut q = std::collections::VecDeque::new();
    for iy in 0..ny {
        for ix in 0..nx {
            if !free[(iy * nx + ix) as usize] {
                d[(iy * nx + ix) as usize] = 0;
                q.push_back((ix, iy));
            }
        }
    }
    while let Some((x, y)) = q.pop_front() {
        let dd = d[(y * nx + x) as usize] + 1;
        for (ox, oy) in [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)] {
            let (x2, y2) = (x + ox, y + oy);
            if x2 < 0 || y2 < 0 || x2 >= nx || y2 >= ny {
                continue;
            }
            let k = (y2 * nx + x2) as usize;
            if d[k] > dd {
                d[k] = dd;
                q.push_back((x2, y2));
            }
        }
    }
    d
}

/// 貪欲ロールアウト。`step` は方策の action id → 実際の遷移 (最頻遷移で共通化)。
/// 戻り値: (到達したか, 歩数, 最小障害物距離)。
fn rollout(
    vi: &ValueIterator,
    step: &[Vec<(i32, i32, i32)>],
    dist: &[i32],
    start: usize,
    path: Option<&mut Vec<(i32, i32)>>,
) -> (bool, usize, i32) {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let mut idx = start;
    let mut min_d = i32::MAX;
    let mut path = path;
    for n in 0..5000 {
        let s = &vi.states[idx];
        if let Some(p) = path.as_deref_mut() {
            p.push((s.ix, s.iy));
        }
        min_d = min_d.min(dist[(s.iy * nx + s.ix) as usize]);
        if s.final_state {
            return (true, n, min_d);
        }
        let Some(a) = s.optimal_action else { return (false, n, min_d) };
        let (dx, dy, dt) = step[a][s.it as usize];
        let (ix, iy, it) = (s.ix + dx, s.iy + dy, (dt + nt) % nt);
        if ix < 0 || iy < 0 || ix >= nx || iy >= ny {
            return (false, n, min_d);
        }
        idx = vi.to_index(ix, iy, it) as usize;
        if !vi.states[idx].free {
            return (false, n, 0);
        }
    }
    (false, 5000, min_d)
}

/// 連続姿勢ロールアウト: セル中心から出発し、方策の行動を `noNoiseStateTransition` で厳密に適用、
/// 着地姿勢のセルで次の方策を読む。戻り値は `rollout` と同じ。
fn rollout_continuous(
    vi: &ValueIterator,
    dist: &[i32],
    start: usize,
    path: Option<&mut Vec<(i32, i32)>>,
) -> (bool, usize, i32) {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let (r, tr) = (vi.xy_resolution, vi.t_resolution);
    let s0 = &vi.states[start];
    let (mut x, mut y, mut t) = ((s0.ix as f64 + 0.5) * r, (s0.iy as f64 + 0.5) * r, (s0.it as f64 + 0.5) * tr);
    let mut min_d = i32::MAX;
    let mut path = path;
    for n in 0..5000 {
        let (ix, iy) = ((x / r).floor() as i32, (y / r).floor() as i32);
        if ix < 0 || iy < 0 || ix >= nx || iy >= ny {
            return (false, n, min_d);
        }
        let it = (((t / tr).floor() as i32) % nt + nt) % nt;
        let idx = vi.to_index(ix, iy, it) as usize;
        let s = &vi.states[idx];
        if !s.free {
            return (false, n, 0);
        }
        if let Some(p) = path.as_deref_mut() {
            p.push((ix, iy));
        }
        min_d = min_d.min(dist[(iy * nx + ix) as usize]);
        if s.final_state {
            return (true, n, min_d);
        }
        let Some(a) = s.optimal_action else { return (false, n, min_d) };
        let act = &vi.actions[a];
        let ang = t.to_radians();
        x += act.delta_fw * ang.cos();
        y += act.delta_fw * ang.sin();
        t += act.delta_rot;
        while t < 0.0 { t += 360.0; }
        while t >= 360.0 { t -= 360.0; }
    }
    (false, 5000, min_d)
}

fn write_pgm(path: &std::path::Path, w: usize, h: usize, px16: &[u16]) {
    let mut buf = format!("P5\n{w} {h}\n65535\n").into_bytes();
    for v in px16 {
        buf.extend_from_slice(&v.to_be_bytes());
    }
    std::fs::write(path, buf).expect("pgm");
}

/// θ=0 スライスの価値 (秒, 65534 で飽和, 65535 = 到達不能) と方策 (id, 255 = なし) を書く。
fn dump_slices(vi: &ValueIterator, dir: &std::path::Path, tag: &str) {
    let (nx, ny) = (vi.cell_num_x as usize, vi.cell_num_y as usize);
    let mut val = vec![65535u16; nx * ny];
    let mut pol = vec![255u16; nx * ny];
    for iy in 0..ny {
        for ix in 0..nx {
            let s = &vi.states[vi.to_index(ix as i32, iy as i32, 0) as usize];
            if s.total_cost < REACH {
                val[iy * nx + ix] = (s.total_cost / PROB_BASE).min(65534) as u16;
                pol[iy * nx + ix] = s.optimal_action.map_or(254, |a| a as u16);
            }
        }
    }
    write_pgm(&dir.join(format!("{tag}_value.pgm")), nx, ny, &val);
    write_pgm(&dir.join(format!("{tag}_policy.pgm")), nx, ny, &pol);
}


/// 確率モデル上での Q(s,a)。本家 `actionCost`。
fn q_on(vi: &ValueIterator, idx: usize, a: usize) -> u64 {
    let s = &vi.states[idx];
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let mut cost: u64 = 0;
    for tr in &vi.actions[a].state_transitions[s.it as usize] {
        let (ix, iy) = (s.ix + tr.dix, s.iy + tr.diy);
        if ix < 0 || iy < 0 || ix >= nx || iy >= ny {
            return MAX_COST;
        }
        let after = &vi.states[vi.to_index(ix, iy, (tr.dit + nt) % nt) as usize];
        if !after.free {
            return MAX_COST;
        }
        cost = cost.wrapping_add(
            after.total_cost.wrapping_add(after.penalty).wrapping_add(after.local_penalty)
                .wrapping_mul(tr.prob as u64),
        );
    }
    cost >> PROB_BASE_BIT
}

/// 2D 連結成分ラベル (free=true のみ)。戻り値: (label, サイズ)。
fn components(free: &[bool], nx: i32, ny: i32) -> (Vec<i32>, Vec<usize>) {
    let mut lab = vec![-1i32; free.len()];
    let mut sizes = Vec::new();
    for s in 0..free.len() {
        if !free[s] || lab[s] >= 0 {
            continue;
        }
        let id = sizes.len() as i32;
        let mut n = 0;
        let mut st = vec![s];
        lab[s] = id;
        while let Some(k) = st.pop() {
            n += 1;
            let (x, y) = ((k as i32) % nx, (k as i32) / nx);
            for (ox, oy) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
                let (x2, y2) = (x + ox, y + oy);
                if x2 < 0 || y2 < 0 || x2 >= nx || y2 >= ny {
                    continue;
                }
                let k2 = (y2 * nx + x2) as usize;
                if free[k2] && lab[k2] < 0 {
                    lab[k2] = id;
                    st.push(k2);
                }
            }
        }
        sizes.push(n);
    }
    (lab, sizes)
}

fn main() {
    let args = Args::parse();
    let map_path = args.map.clone().unwrap_or_else(|| {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/map_tsudanuma.yaml")
    });
    let solver = U64Solver::from_name(&args.solver).expect("solver");
    let map = pgm::load(&map_path).expect("map");
    let res = map.meta.resolution * args.scale as f64;
    let (occ, ow, oh) = pgm::build_occupancy(&map, args.scale, true);
    let grid = OccupancyGrid {
        width: ow,
        height: oh,
        resolution: res,
        origin_x: map.meta.origin_x,
        origin_y: map.meta.origin_y,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: occ,
    };
    let free2d: Vec<bool> = grid.data.iter().map(|&d| d == 0).collect();
    let (lab, sizes) = components(&free2d, ow, oh);
    let dist2d = obstacle_dist(&free2d, ow, oh);
    let main_comp = (0..sizes.len()).max_by_key(|&i| sizes[i]).unwrap() as i32;
    let to_world = |ix: i32, iy: i32| {
        (map.meta.origin_x + (ix as f64 + 0.5) * res, map.meta.origin_y + (iy as f64 + 0.5) * res)
    };
    // ゴール: 指定 or 4 象限それぞれで最大成分に最も近い free セル。
    let goals: Vec<(f64, f64)> = match &args.goals {
        Some(g) => g
            .split(';')
            .map(|p| {
                let mut it = p.split(',').map(|v| v.trim().parse::<f64>().expect("goal"));
                (it.next().unwrap(), it.next().unwrap())
            })
            .collect(),
        None => [(0.25, 0.25), (0.75, 0.25), (0.25, 0.75), (0.75, 0.75)]
            .iter()
            .map(|&(fx, fy)| {
                let (cx, cy) = ((ow as f64 * fx) as i32, (oh as f64 * fy) as i32);
                let mut best = (i64::MAX, 0, 0);
                for iy in 0..oh {
                    for ix in 0..ow {
                        // 壁から 1 m 以上離れたセルに置く (壁際は確率モデルだと袋小路になる)。
                        if lab[(iy * ow + ix) as usize] == main_comp && dist2d[(iy * ow + ix) as usize] as f64 * res >= 1.0 {
                            let d = ((ix - cx) as i64).pow(2) + ((iy - cy) as i64).pow(2);
                            if d < best.0 {
                                best = (d, ix, iy);
                            }
                        }
                    }
                }
                to_world(best.1, best.2)
            })
            .collect(),
    };
    eprintln!("grid {}x{}x{} @ {:.3} m/cell, main component {} cells, goals {:?}", ow, oh, N_THETA, res, sizes[main_comp as usize], goals);

    let build = |mode: &str, gx: f64, gy: f64| -> (ValueIterator, f64) {
        // spec 解析: base[_g][_pN]
        let base_name;
        let mut graded = false;
        let mut pen = args.safety_penalty;
        if mode == "graded" { base_name = "center"; graded = true; }
        else {
            let mut parts = mode.split('_');
            base_name = parts.next().unwrap();
            for t in parts {
                if t == "g" { graded = true; }
                else if let Some(v) = t.strip_prefix('p') { pen = v.parse().expect("_pN"); }
                else { panic!("unknown model suffix {t} in {mode}"); }
            }
        }
        let mut vi = ValueIterator::new(scaled_actions(args.action_scale), 1);
        vi.set_map_with_occupancy_grid(&grid, N_THETA, args.safety_radius_m, pen, args.goal_radius_m, 15);
        match base_name {
            "center" => make_center(&mut vi),
            "mode" => make_mode(&mut vi),
            "stochastic" => {}
            _ => panic!("unknown model {mode}"),
        }
        if graded {
            let margin = (args.safety_radius_m / res).ceil() as i32;
            make_graded(&mut vi, &dist2d, margin, (args.ramp_m / res).max(1e-9), pen);
        }
        vi.set_goal(gx, gy, 0);
        let t = std::time::Instant::now();
        let st = solve(&mut vi, solver, 100_000);
        let secs = t.elapsed().as_secs_f64();
        eprintln!("{mode:>10}: solve {secs:.2}s iters {} converged {}", st.iters, st.converged);
        (vi, secs)
    };

    let mut json_goals = Vec::new();
    for (gi, &(gx, gy)) in goals.iter().enumerate() {
        println!("== goal {gi}: ({gx:.2}, {gy:.2})");
        let (base, base_secs) = build("stochastic", gx, gy);
        let step: Vec<Vec<(i32, i32, i32)>> = base
            .actions
            .iter()
            .map(|a| a.state_transitions.iter().map(|l| { let m = l.iter().max_by_key(|s| s.prob).unwrap(); (m.dix, m.diy, m.dit) }).collect())
            .collect();
        let dist = obstacle_dist(&free2d, ow, oh);
        let reach_base: Vec<usize> = (0..base.states.len()).filter(|&i| base.states[i].total_cost < REACH && !base.states[i].final_state).collect();
        let mut seed = 0x9E3779B97F4A7C15u64 ^ gi as u64;
        let starts: Vec<usize> = (0..args.rollouts)
            .map(|_| { seed ^= seed << 13; seed ^= seed >> 7; seed ^= seed << 17; reach_base[(seed % reach_base.len() as u64) as usize] })
            .collect();
        let roll = |v: &ValueIterator| {
            let (mut ok, mut steps, mut mind, mut coll) = (0usize, 0usize, Vec::new(), 0usize);
            for &s in &starts {
                let (r, st, d) = if args.exec == "modal" { rollout(v, &step, &dist, s, None) } else { rollout_continuous(v, &dist, s, None) };
                ok += r as usize; steps += st; mind.push(d); coll += (d == 0) as usize;
            }
            mind.sort();
            (ok, steps as f64 / starts.len() as f64, mind[0], mind[mind.len() / 2], coll)
        };
        let dump = args.dump_dir.as_ref().map(|d| { let d = d.join(format!("goal{gi}")); std::fs::create_dir_all(&d).unwrap(); d });
        let path_starts: Vec<usize> = starts.iter().copied().take(args.path_starts).collect();
        let mut paths_json: Vec<String> = Vec::new();
        let mut paths_of = |v: &ValueIterator, tag: &str| {
            let ps: Vec<String> = path_starts.iter().map(|&s| {
                let mut p = Vec::new();
                if args.exec == "modal" { rollout(v, &step, &dist, s, Some(&mut p)); } else { rollout_continuous(v, &dist, s, Some(&mut p)); }
                format!("[{}]", p.iter().map(|(x, y)| format!("[{x},{y}]")).collect::<Vec<_>>().join(","))
            }).collect();
            paths_json.push(format!(r#""{tag}":[{}]"#, ps.join(",")));
        };
        if let Some(d) = &dump {
            let free: Vec<u16> = (0..(ow * oh) as usize).map(|k| if free2d[k] { 65535 } else { 0 }).collect();
            write_pgm(&d.join("free.pgm"), ow as usize, oh as usize, &free);
            dump_slices(&base, d, "stochastic");
            paths_of(&base, "stochastic");
        }
        let rb = roll(&base);
        println!("stochastic: solve {base_secs:.1}s reachable {} rollout reached {}/{} steps {:.1} clr min {} med {} coll {}", reach_base.len(), rb.0, starts.len(), rb.1, rb.2, rb.3, rb.4);
        let mut json_models = vec![format!(
            r#"{{"name":"stochastic","solve_s":{base_secs:.2},"reachable":{},"reached":{},"steps":{:.2},"clr_min":{},"clr_med":{},"collisions":{}}}"#,
            reach_base.len(), rb.0, rb.1, rb.2, rb.3, rb.4)];
        let mut mism_grid = String::new();
        for mode in args.models.split(',').map(str::trim) {
            let (v, secs) = build(mode, gx, gy);
            let (mut n, mut sum_abs, mut max_abs, mut sum_rel, mut mism, mut regret_sum, mut regret_max, mut regret_n) =
                (0usize, 0f64, 0f64, 0f64, 0usize, 0f64, 0f64, 0usize);
            let (gw, gh) = (ow as usize / args.grid_step, oh as usize / args.grid_step);
            let mut mg = vec![0u32; gw * gh];
            let mut mgn = vec![0u32; gw * gh];
            for &i in &reach_base {
                let s = &v.states[i];
                if s.total_cost >= REACH { continue; }
                n += 1;
                let (a, b) = (base.states[i].total_cost as f64, s.total_cost as f64);
                let d = (b - a).abs();
                sum_abs += d; max_abs = max_abs.max(d); sum_rel += d / a.max(1.0);
                let bs = &base.states[i];
                let cell = (bs.iy as usize / args.grid_step).min(gh - 1) * gw + (bs.ix as usize / args.grid_step).min(gw - 1);
                mgn[cell] += 1;
                if bs.optimal_action != s.optimal_action {
                    mism += 1;
                    mg[cell] += 1;
                    if let Some(ad) = s.optimal_action {
                        let q = q_on(&base, i, ad);
                        if q < MAX_COST {
                            let r = (q as f64 - bs.total_cost as f64).max(0.0);
                            regret_sum += r; regret_max = regret_max.max(r); regret_n += 1;
                        } else {
                            regret_n += 1; regret_max = f64::INFINITY;
                        }
                    }
                }
            }
            let k = n as f64;
            let rr = roll(&v);
            if let Some(d) = &dump {
                dump_slices(&v, d, mode);
                paths_of(&v, mode);
            }
            println!(
                "{mode:>10}: solve {secs:.1}s |dV| mean {:.3}s max {:.3}s rel {:.2}% mismatch {:.2}% regret mean {:.3}s max {:.1}s (n {}) rollout reached {}/{} steps {:.1} clr min {} med {} coll {}",
                sum_abs / k / PROB_BASE as f64, max_abs / PROB_BASE as f64, sum_rel / k * 100.0, mism as f64 / k * 100.0,
                regret_sum / mism.max(1) as f64 / PROB_BASE as f64, regret_max / PROB_BASE as f64, regret_n,
                rr.0, starts.len(), rr.1, rr.2, rr.3, rr.4);
            json_models.push(format!(
                r#"{{"name":"{mode}","solve_s":{secs:.2},"reachable":{n},"dv_mean_s":{:.4},"dv_max_s":{:.3},"dv_rel_pct":{:.3},"mismatch_pct":{:.3},"regret_mean_s":{:.4},"regret_max_s":{},"reached":{},"steps":{:.2},"clr_min":{},"clr_med":{},"collisions":{}}}"#,
                sum_abs / k / PROB_BASE as f64, max_abs / PROB_BASE as f64, sum_rel / k * 100.0, mism as f64 / k * 100.0,
                regret_sum / mism.max(1) as f64 / PROB_BASE as f64,
                if regret_max.is_finite() { format!("{:.2}", regret_max / PROB_BASE as f64) } else { "null".into() },
                rr.0, rr.1, rr.2, rr.3, rr.4));
            if let Some(d) = &dump {
                let (mut mg2, mut mgn2) = (vec![0u32; (ow * oh) as usize], vec![0u32; (ow * oh) as usize]);
                for &i in &reach_base {
                    let bs = &base.states[i];
                    if v.states[i].total_cost >= REACH { continue; }
                    let c = (bs.iy * ow + bs.ix) as usize;
                    mgn2[c] += 1;
                    mg2[c] += (bs.optimal_action != v.states[i].optimal_action) as u32;
                }
                let px: Vec<u16> = (0..mgn2.len()).map(|c| if mgn2[c] == 0 { 65535 } else { (mg2[c] * 100 / mgn2[c]) as u16 }).collect();
                write_pgm(&d.join(format!("{mode}_mismatch.pgm")), ow as usize, oh as usize, &px);
            }
            if mism_grid.is_empty() {
                // 0..100 の不一致率 (θ平均)、-1 = 到達不能。
                let vals: Vec<String> = (0..gw * gh).map(|c| if mgn[c] == 0 { "-1".into() } else { format!("{}", mg[c] * 100 / mgn[c]) }).collect();
                mism_grid = format!(r#""grid":{{"w":{gw},"h":{gh},"step_m":{:.3},"mismatch":[{}]}}"#, res * args.grid_step as f64, vals.join(","));
            }
        }
        if let Some(d) = &dump {
            let (gix, giy) = (((gx - map.meta.origin_x) / res) as i32, ((gy - map.meta.origin_y) / res) as i32);
            std::fs::write(d.join("paths.json"), format!(r#"{{"goal_cell":[{gix},{giy}],{}}}"#, paths_json.join(","))).unwrap();
        }
        json_goals.push(format!(r#"{{"goal":[{gx:.2},{gy:.2}],"models":[{}],{}}}"#, json_models.join(","), mism_grid));
    }
    if let Some(out) = &args.out {
        std::fs::write(out, format!(r#"{{"map":"{}","res_m":{res:.3},"w":{ow},"h":{oh},"origin":[{:.2},{:.2}],"safety_radius_m":{},"goals":[{}]}}"#,
            map_path.file_name().unwrap().to_string_lossy(), map.meta.origin_x, map.meta.origin_y, args.safety_radius_m, json_goals.join(","))).expect("write");
    }
}
