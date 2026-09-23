//! `schur_demo` — SchurPortal のインタラクティブデモ。
//!
//! 地図とアーティファクトを常駐させ、ブラウザ GUI (canvas、パン/ズーム対応)
//! から 右クリック = ゴール / 左クリック = スタートを受ける。勾配は 3 段階で
//! プログレッシブに正確になる:
//!
//! 1. ゴール確定 → ポータル Dijkstra (~0.3-0.5 s) → **タイル粒度の粗い勾配を
//!    即時表示** (各タイル = 所属ポータルの min D)。
//! 2. 全場 V̂ をタイル並列復元 → セル粒度の上界勾配 (min over θ)。
//! 3. V̂ 温間の exactify (並列 frontier2d_par_unsafe) → **厳密場** に置き換え。
//! 4. スタート確定 → 現在の場の greedy rollout → 経路をオーバーレイ。
//!
//! HTTP は std のみ (依存追加なし)。状態は Mutex<Demo> で逐次処理。
//!
//!   # tb3_house (小、全段が秒未満〜2 s)
//!   cargo run --release -p vi_bench --bin schur_demo
//!   # 津田沼キャンパス (157M 状態: 粗 ~0.4 s / V̂ ~40 s / 厳密 ~+21 s)
//!   cargo run --release -p vi_bench --bin schur_demo -- \
//!       --map ../assets/map_tsudanuma.yaml --scale 3
//!   → http://127.0.0.1:8008 を開く
//!
//! アーティファクトは `--out` (既定 schur_demo_out/) にキャッシュされ、
//! 2 回目以降の起動は即時。

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::sync::Mutex;
use std::time::Instant;

use clap::Parser;

use vi_bench::params::{canonical_actions, N_THETA};
use vi_bench::pgm;
use vi_lib::params::{MAX_COST, PROB_BASE};
use vi_lib::planner::rollout_path_on;
use vi_lib::solvers::schur::{
    build, fix_portal_actions, materialize_many, portal_costs, SchurArtifact, SchurConfig,
};
use vi_lib::solvers::{solve, U64Solver, REACH_THRESH as REACH};
use vi_lib::{OccupancyGrid, Quaternion, State, ValueIterator};

#[derive(Parser)]
#[command(about = "Interactive SchurPortal demo: click start/goal in the browser, see the gradient.")]
struct Args {
    /// Map YAML。既定は tb3_house。
    #[arg(long)]
    map: Option<PathBuf>,
    #[arg(long, default_value_t = 2)]
    scale: usize,
    /// 提供するタイル一辺の候補 (カンマ区切り)。キャッシュ済み成果物があるものだけ
    /// ドロップダウンに載る (無いものはスキップ。大きい地図の成果物は schur_bench
    /// --gpu で作ってから)。1 つも無ければ先頭のタイルを CPU ビルドする。
    #[arg(long, default_value = "32,48,64,96,128,192,256")]
    tiles: String,
    #[arg(long, default_value_t = 8)]
    thetas: i32,
    #[arg(long, default_value_t = 0.2)]
    safety_radius_m: f64,
    #[arg(long, default_value_t = 30.0)]
    safety_penalty: f64,
    /// アーティファクトのキャッシュ先。
    #[arg(long, default_value = "schur_demo_out")]
    out: PathBuf,
    #[arg(long, default_value_t = 8008)]
    port: u16,
}

fn default_map_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/tb3_house/map.yaml")
}

/// schur_bench と同じ sweep_orders なしの states 構築 (行バンド並列)。
fn build_states(grid: &OccupancyGrid, nt: i32, safety_radius: f64, penalty: f64) -> Vec<State> {
    let margin = (safety_radius / grid.resolution).ceil() as i32;
    let (nx, ny) = (grid.width, grid.height);
    let nthr = std::thread::available_parallelism().map(|p| p.get()).unwrap_or(1);
    let rows: Vec<i32> = (0..ny).collect();
    let chunk = rows.len().div_ceil(nthr).max(1);
    let bands: Vec<Vec<State>> = std::thread::scope(|scope| {
        let handles: Vec<_> = rows
            .chunks(chunk)
            .map(|band| {
                scope.spawn(move || {
                    let mut out = Vec::with_capacity(band.len() * nx as usize * nt as usize);
                    for &y in band {
                        for x in 0..nx {
                            for t in 0..nt {
                                out.push(State::from_occupancy(x, y, t, grid, margin, penalty, nx));
                            }
                        }
                    }
                    out
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });
    bands.concat()
}

fn snap_to_free(occ: &[i8], w: i32, h: i32, gx: i32, gy: i32, max_r: i32) -> Option<(i32, i32)> {
    let at = |x: i32, y: i32| (y * w + x) as usize;
    if gx >= 0 && gx < w && gy >= 0 && gy < h && occ[at(gx, gy)] == 0 {
        return Some((gx, gy));
    }
    for r in 1..=max_r {
        for dy in -r..=r {
            for dx in -r..=r {
                if dx.abs() != r && dy.abs() != r {
                    continue;
                }
                let (x, y) = (gx + dx, gy + dy);
                if x >= 0 && y >= 0 && x < w && y < h && occ[at(x, y)] == 0 {
                    return Some((x, y));
                }
            }
        }
    }
    None
}

struct Demo {
    vi: ValueIterator,
    /// タイル一辺 → 成果物。`cur` が選択中。
    arts: std::collections::BTreeMap<i32, SchurArtifact>,
    cur: i32,
    grid: OccupancyGrid,
    goal: Option<(i32, i32)>,
    d: Option<Vec<u64>>,
    /// ゴール確定時に立てた実 final (materialize_one が釘に使う)。
    finals: Vec<(i32, i32, i32)>,
    /// 未復元タイル (末尾 = 次 = ゴールに近い順)。
    pending: Vec<i32>,
    total: usize,
    /// ゴール/タイル切替の世代。古い /field_step を弾く。
    gen: u64,
    /// 全場 V̂ が組み上がっているか / さらに厳密化済みか。
    materialized: bool,
    exact: bool,
}

impl Demo {
    fn art(&self) -> &SchurArtifact {
        &self.arts[&self.cur]
    }

    /// ゴール確定/タイル切替の共通経路: set_goal → Dijkstra → タイル粒度の粗勾配 JSON。
    fn run_goal(&mut self, gx: i32, gy: i32) -> String {
        let (wx, wy) = self.world_of(gx, gy);
        let t0 = Instant::now();
        self.vi.set_goal(wx, wy, 90);
        let dd = portal_costs(&self.vi, &self.arts[&self.cur]);
        let ms = t0.elapsed().as_secs_f64() * 1e3;
        let art = self.art();
        let mut coarse = vec![-1.0f64; art.tiles.len()];
        for (t, e) in art.tiles.iter().enumerate() {
            let m = e.portals.iter().map(|&p| dd[p as usize]).min().unwrap_or(MAX_COST);
            if m < REACH {
                coarse[t] = m as f64 / PROB_BASE as f64;
            }
        }
        let json = format!(
            "{{\"ms\":{ms:.1},\"gx\":{gx},\"gy\":{gy},\"tile\":{},\"tnx\":{},\"tny\":{},\"coarse\":[{}]}}",
            art.tile,
            art.tnx,
            art.tny,
            coarse
                .iter()
                .map(|v| if *v < 0.0 { "-1".to_string() } else { format!("{v:.1}") })
                .collect::<Vec<_>>()
                .join(",")
        );
        // 復元キュー: ポータル D の最小が小さい (= ゴールに近い) タイルから。
        // ポータルが全滅でも実 final を含むタイルは先頭 (ゴールタイル自身)。
        let finals: Vec<(i32, i32, i32)> = self
            .vi
            .states
            .iter()
            .filter(|s| s.final_state)
            .map(|s| (s.ix, s.iy, s.it))
            .collect();
        let art = &self.arts[&self.cur];
        let side = art.tile + 2 * art.halo;
        let mut keyed: Vec<(u64, i32)> = Vec::new();
        for t in 0..art.tiles.len() as i32 {
            let e = &art.tiles[t as usize];
            let mut k = e.portals.iter().map(|&p| dd[p as usize]).min().unwrap_or(MAX_COST);
            if k >= MAX_COST {
                let (tx, ty) = (t % art.tnx, t / art.tnx);
                let (x0, y0) = (tx * art.tile - art.halo, ty * art.tile - art.halo);
                if finals
                    .iter()
                    .any(|&(fx, fy, _)| fx >= x0 && fx < x0 + side && fy >= y0 && fy < y0 + side)
                {
                    k = 0;
                }
            }
            if k < MAX_COST {
                keyed.push((k, t));
            }
        }
        keyed.sort();
        self.pending = keyed.into_iter().map(|(_, t)| t).rev().collect(); // pop() = 近い順
        self.total = self.pending.len();
        self.finals = finals;
        self.gen += 1;
        self.d = Some(dd);
        self.goal = Some((gx, gy));
        self.materialized = false;
        self.exact = false;
        format!("{{\"gen\":{},\"total\":{},{}", self.gen, self.total, &json[1..])
    }
    fn world_of(&self, ix: i32, iy: i32) -> (f64, f64) {
        (
            self.grid.origin_x + (ix as f64 + 0.5) * self.grid.resolution,
            self.grid.origin_y + (iy as f64 + 0.5) * self.grid.resolution,
        )
    }

    /// 現在の場の min-over-θ [秒] を f32 LE で。先頭 4 byte は計測 ms。
    fn field_bytes(&self, ms: f64) -> Vec<u8> {
        let (nx, ny, nt) = (self.grid.width, self.grid.height, N_THETA);
        let mut bytes = Vec::with_capacity((nx * ny * 4) as usize + 4);
        bytes.extend_from_slice(&(ms as f32).to_le_bytes());
        for iy in 0..ny {
            for ix in 0..nx {
                let base = (ix * nt + iy * nt * nx) as usize;
                let mut m = MAX_COST;
                for it in 0..nt as usize {
                    m = m.min(self.vi.states[base + it].total_cost);
                }
                let v = if m >= REACH { f32::INFINITY } else { m as f32 / PROB_BASE as f32 };
                bytes.extend_from_slice(&v.to_le_bytes());
            }
        }
        bytes
    }
}

fn main() {
    let args = Args::parse();
    let map_path = args.map.clone().unwrap_or_else(default_map_path);
    std::fs::create_dir_all(&args.out).expect("mkdir out");

    eprintln!("loading map: {}", map_path.display());
    let map = pgm::load(&map_path).expect("load map");
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
    let goal_radius_m = (2.0 * res).max(0.5);

    eprintln!("building states: {}x{}x{} ...", ow, oh, N_THETA);
    let mut vi = ValueIterator::new(canonical_actions(), 1);
    vi.set_map_geometry_no_states(&grid, N_THETA, goal_radius_m, 15);
    vi.states = build_states(&grid, N_THETA, args.safety_radius_m, args.safety_penalty);

    let stem = map_path.file_stem().and_then(|s| s.to_str()).unwrap_or("map").to_string();
    let tile_list: Vec<i32> = args.tiles.split(',').filter_map(|t| t.trim().parse().ok()).collect();
    let mut arts: std::collections::BTreeMap<i32, SchurArtifact> = std::collections::BTreeMap::new();
    for &t in &tile_list {
        let path = args.out.join(format!("schur_demo_{stem}_s{}_t{t}_h{}.bin", args.scale, args.thetas));
        if path.exists() {
            let a = SchurArtifact::load(&path).expect("load artifact");
            if (a.nx, a.ny) != (ow, oh) {
                eprintln!("skip tile {t}: 成果物の格子 {}x{} が地図 {ow}x{oh} と不一致", a.nx, a.ny);
                continue;
            }
            eprintln!("tile {t}: loaded {} ({} portals, {:.1} MB)", path.display(), a.portals.len(), a.size_bytes() as f64 / 1e6);
            arts.insert(t, a);
        } else {
            eprintln!("tile {t}: 成果物なし ({}) — スキップ", path.display());
        }
    }
    if arts.is_empty() {
        let t = *tile_list.first().expect("--tiles が空");
        eprintln!("成果物が 1 つも無いので tile {t} を CPU ビルド (1 回だけ、次回からキャッシュ) ...");
        let t0 = Instant::now();
        let a = build(&vi, &SchurConfig { tile: t, portal_thetas: args.thetas, spacing: 16 });
        eprintln!("artifact built in {:.1} s ({} portals)", t0.elapsed().as_secs_f64(), a.portals.len());
        a.save(&args.out.join(format!("schur_demo_{stem}_s{}_t{t}_h{}.bin", args.scale, args.thetas)))
            .expect("save artifact");
        arts.insert(t, a);
    }
    let cur = *arts.keys().next().unwrap();
    eprintln!("ready: tiles {:?}, 選択中 {cur}", arts.keys().collect::<Vec<_>>());

    let demo = Mutex::new(Demo {
        vi,
        arts,
        cur,
        grid,
        goal: None,
        d: None,
        finals: Vec::new(),
        pending: Vec::new(),
        total: 0,
        gen: 0,
        materialized: false,
        exact: false,
    });

    let listener = TcpListener::bind(("127.0.0.1", args.port)).expect("bind");
    eprintln!("open http://127.0.0.1:{}/  (右クリック=ゴール, 左クリック=スタート, ドラッグ=パン, ホイール=ズーム)", args.port);
    for stream in listener.incoming() {
        let Ok(stream) = stream else { continue };
        if let Err(e) = handle(stream, &demo) {
            eprintln!("request error: {e}");
        }
    }
}

// ── 最小 HTTP ────────────────────────────────────────────────────────────────

fn handle(mut stream: TcpStream, demo: &Mutex<Demo>) -> std::io::Result<()> {
    let mut buf = [0u8; 4096];
    let n = stream.read(&mut buf)?;
    let req = String::from_utf8_lossy(&buf[..n]);
    let line = req.lines().next().unwrap_or("");
    let path = line.split_whitespace().nth(1).unwrap_or("/");
    let (route, query) = match path.split_once('?') {
        Some((r, q)) => (r, q),
        None => (path, ""),
    };
    let q = |key: &str| -> Option<f64> {
        query
            .split('&')
            .find_map(|kv| kv.strip_prefix(&format!("{key}=")))
            .and_then(|v| v.parse().ok())
    };

    match route {
        "/" => respond(&mut stream, "text/html; charset=utf-8", HTML.as_bytes()),
        "/meta" => {
            let d = demo.lock().unwrap();
            let portals: Vec<String> = {
                let mut seen = std::collections::HashSet::new();
                d.art()
                    .portals
                    .iter()
                    .filter(|p| seen.insert((p.ix, p.iy)))
                    .map(|p| format!("[{},{}]", p.ix, p.iy))
                    .collect()
            };
            let json = format!(
                "{{\"w\":{},\"h\":{},\"res\":{},\"tile\":{},\"tnx\":{},\"tny\":{},\"tiles\":[{}],\"portals\":[{}]}}",
                d.grid.width,
                d.grid.height,
                d.grid.resolution,
                d.art().tile,
                d.art().tnx,
                d.art().tny,
                d.arts.keys().map(|t| t.to_string()).collect::<Vec<_>>().join(","),
                portals.join(",")
            );
            respond(&mut stream, "application/json", json.as_bytes())
        }
        "/map" => {
            let d = demo.lock().unwrap();
            let bytes: Vec<u8> = d.grid.data.iter().map(|&c| if c == 0 { 0u8 } else { 1 }).collect();
            respond(&mut stream, "application/octet-stream", &bytes)
        }
        // ゴール確定: set_goal + ポータル Dijkstra → タイル粒度の粗い勾配 [秒]。
        "/goal" => {
            let (ix, iy) = (q("x").unwrap_or(0.0) as i32, q("y").unwrap_or(0.0) as i32);
            let mut d = demo.lock().unwrap();
            let Some((gx, gy)) =
                snap_to_free(&d.grid.data, d.grid.width, d.grid.height, ix, iy, 200)
            else {
                return respond(&mut stream, "application/json", b"{\"err\":\"no free cell near goal\"}");
            };
            let json = d.run_goal(gx, gy);
            respond(&mut stream, "application/json", json.as_bytes())
        }
        // タイル切替: ゴールが立っていれば新しい成果物で粗勾配から取り直す。
        "/tile" => {
            let t = q("t").unwrap_or(0.0) as i32;
            let mut d = demo.lock().unwrap();
            if !d.arts.contains_key(&t) {
                return respond(&mut stream, "application/json", b"{\"err\":\"unknown tile\"}");
            }
            d.cur = t;
            d.materialized = false;
            d.exact = false;
            let json = if let Some((gx, gy)) = d.goal {
                d.run_goal(gx, gy)
            } else {
                let art = d.art();
                format!("{{\"tile\":{},\"tnx\":{},\"tny\":{}}}", art.tile, art.tnx, art.tny)
            };
            respond(&mut stream, "application/json", json.as_bytes())
        }
        // 現在場のスナップショット (min-over-θ)。復元は /field_step が進める。
        "/field" => {
            let d = demo.lock().unwrap();
            if d.goal.is_none() {
                return respond(&mut stream, "application/json", b"{\"err\":\"set goal first\"}");
            }
            let t0 = Instant::now();
            let bytes = d.field_bytes(t0.elapsed().as_secs_f64() * 1e3);
            respond(&mut stream, "application/octet-stream", &bytes)
        }
        // 漸進復元: ~0.4 s ぶんゴールに近いタイルから materialize して進捗を返す。
        "/field_step" => {
            let gen = q("gen").unwrap_or(-1.0) as u64;
            let mut dm = demo.lock().unwrap();
            if dm.goal.is_none() || gen != dm.gen {
                return respond(&mut stream, "application/json", b"{\"err\":\"stale\"}");
            }
            let t0 = Instant::now();
            let cur = dm.cur;
            // ゴールに近い側から 1 バッチをスレッド並列で復元 (~0.1-0.5 s)。
            const BATCH: usize = 48;
            let take = BATCH.min(dm.pending.len());
            let cut = dm.pending.len() - take;
            let batch: Vec<i32> = dm.pending.split_off(cut);
            let n = batch.len();
            {
                let Demo { vi, arts, d: dd, finals, .. } = &mut *dm;
                materialize_many(vi, &arts[&cur], dd.as_ref().unwrap(), finals, &batch);
            }
            if dm.pending.is_empty() && !dm.materialized {
                dm.materialized = true;
                let Demo { vi, arts, .. } = &mut *dm;
                fix_portal_actions(vi, &arts[&cur]);
            }
            let done = dm.total - dm.pending.len();
            let json = format!(
                "{{\"done\":{done},\"total\":{},\"finished\":{},\"ms\":{:.1},\"n\":{n}}}",
                dm.total,
                dm.materialized,
                t0.elapsed().as_secs_f64() * 1e3
            );
            respond(&mut stream, "application/json", json.as_bytes())
        }
        // V̂ 温間の exactify → 厳密場 (収束した Bellman 固定点) を返す。
        "/exact" => {
            let mut d = demo.lock().unwrap();
            if !d.materialized {
                return respond(&mut stream, "application/json", b"{\"err\":\"field not ready\"}");
            }
            let t0 = Instant::now();
            if !d.exact {
                let st = solve(&mut d.vi, U64Solver::Frontier2DParUnsafe, 1_000_000);
                if !st.converged {
                    return respond(&mut stream, "application/json", b"{\"err\":\"exactify did not converge\"}");
                }
                d.exact = true;
            }
            let ms = t0.elapsed().as_secs_f64() * 1e3;
            let bytes = d.field_bytes(ms);
            respond(&mut stream, "application/octet-stream", &bytes)
        }
        // スタート確定: 現在の場の greedy rollout → セル座標の折れ線。
        "/path" => {
            let d = demo.lock().unwrap();
            if d.goal.is_none() || d.total == d.pending.len() {
                return respond(&mut stream, "application/json", b"{\"err\":\"field not ready\"}");
            }
            let (ix, iy) = (q("x").unwrap_or(0.0) as i32, q("y").unwrap_or(0.0) as i32);
            let th = q("th").unwrap_or(0.0);
            let Some((sx, sy)) =
                snap_to_free(&d.grid.data, d.grid.width, d.grid.height, ix, iy, 200)
            else {
                return respond(&mut stream, "application/json", b"{\"err\":\"no free cell near start\"}");
            };
            let (wx, wy) = d.world_of(sx, sy);
            let t0 = Instant::now();
            let r = rollout_path_on(&d.vi, wx, wy, th.to_radians(), 200_000, 2);
            let ms = t0.elapsed().as_secs_f64() * 1e3;
            let pts: Vec<String> = r
                .poses
                .iter()
                .map(|p| {
                    format!(
                        "[{:.2},{:.2}]",
                        (p.x - d.grid.origin_x) / d.grid.resolution,
                        (p.y - d.grid.origin_y) / d.grid.resolution
                    )
                })
                .collect();
            let json = format!(
                "{{\"ms\":{ms:.1},\"status\":\"{:?}\",\"reached\":{},\"exact\":{},\"pts\":[{}]}}",
                r.status,
                r.reached_goal(),
                d.exact,
                pts.join(",")
            );
            respond(&mut stream, "application/json", json.as_bytes())
        }
        _ => respond_status(&mut stream, 404, b"not found"),
    }
}

fn respond(stream: &mut TcpStream, ctype: &str, body: &[u8]) -> std::io::Result<()> {
    let head = format!(
        "HTTP/1.1 200 OK\r\nContent-Type: {ctype}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
        body.len()
    );
    stream.write_all(head.as_bytes())?;
    stream.write_all(body)?;
    stream.flush()
}

fn respond_status(stream: &mut TcpStream, code: u16, body: &[u8]) -> std::io::Result<()> {
    let head = format!(
        "HTTP/1.1 {code} X\r\nContent-Type: text/plain\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        body.len()
    );
    stream.write_all(head.as_bytes())?;
    stream.write_all(body)?;
    stream.flush()
}

// ── ブラウザ GUI (自己完結、依存なし、パン/ズーム対応) ─────────────────────

const HTML: &str = r#"<!doctype html><html><head><meta charset="utf-8">
<title>SchurPortal demo</title>
<style>
html,body{margin:0;height:100%;background:#17181b;color:#ecebe6;font-family:"IBM Plex Sans JP","Hiragino Sans",sans-serif;font-size:14px;overflow:hidden}
#bar{display:flex;gap:1em;align-items:center;padding:8px 14px;border-bottom:1px solid #33363c;flex-wrap:wrap}
#bar b{color:#35c08d}
.badge{font-family:ui-monospace,monospace;background:#24262b;border-radius:6px;padding:2px 8px;white-space:nowrap}
.badge.on{background:#1d2f28;color:#35c08d}
canvas{display:block;cursor:crosshair}
select,button{background:#24262b;color:#ecebe6;border:1px solid #33363c;border-radius:6px;padding:2px 6px;font-size:13px}
button{cursor:pointer}button:hover{background:#2c2f35}button:disabled{color:#5c6068;cursor:default}
.hint{color:#a8acb4}
#stage{font-weight:700}
</style></head><body>
<div id="bar">
  <span><b>SchurPortal</b> デモ</span>
  <span class="hint">右クリック=ゴール / 左クリック=スタート / ドラッグ=パン / ホイール=ズーム</span>
  <label>tile <select id="tile"></select></label>
  <button id="ex" disabled>厳密化</button>
  <label>開始方位 <select id="th"><option>0</option><option>45</option><option selected>90</option><option>135</option><option>180</option><option>225</option><option>270</option><option>315</option></select>°</label>
  <span class="badge" id="t_dij">Dijkstra: –</span>
  <span class="badge" id="t_field">V̂ 上界: –</span>
  <span class="badge" id="t_exact">厳密化: –</span>
  <span class="badge" id="t_path">rollout: –</span>
  <span class="badge on" id="stage">場: なし</span>
  <span id="msg" class="hint"></span>
</div>
<canvas id="cv"></canvas>
<script>
const cv = document.getElementById('cv'), ctx = cv.getContext('2d');
let meta, mapbits, field = null, coarse = null, goal = null, start = null, path = null;
let stage = 'なし';
// ビュー変換: screen = cell * s + t (セル座標は y 上向き → 描画時に h-1-iy)
let view = {s: 1, tx: 0, ty: 0};

function viridis(t){
  const s=[[68,1,84],[59,82,139],[33,145,140],[94,201,98],[253,231,37]];
  t=Math.max(0,Math.min(1,t))*4; const i=Math.min(3,Math.floor(t)), f=t-i;
  return s[i].map((a,k)=>Math.round(a+(s[i+1][k]-a)*f));
}

// ── ラスタ (1px/セル) — データ変更時だけ再構築 ──
const off = document.createElement('canvas');
function renderRaster(){
  const {w,h}=meta; off.width=w; off.height=h;
  const img=off.getContext('2d').createImageData(w,h);
  let p99=1;
  if(field){const fin=[]; for(let i=0;i<w*h;i++){const v=field[i]; if(isFinite(v)&&v>0)fin.push(v);}
    fin.sort((a,b)=>a-b); p99=fin[Math.floor(fin.length*0.99)]||1;}
  let c99=1;
  if(coarse){const fin=coarse.filter(v=>v>=0).sort((a,b)=>a-b); c99=fin[Math.floor(fin.length*0.99)]||1;}
  for(let iy=0;iy<h;iy++)for(let ix=0;ix<w;ix++){
    const gi=iy*w+ix, pi=((h-1-iy)*w+ix)*4;
    let r=20,g=20,b=24;
    if(mapbits[gi]===1){r=g=b=8;}
    else if(field){const v=field[gi];
      if(isFinite(v)){const c=viridis(1-v/p99); r=c[0];g=c[1];b=c[2];}
      else {r=45;g=45;b=52;}
    } else if(coarse){
      const t=Math.floor(ix/meta.tile)+Math.floor(iy/meta.tile)*meta.tnx;
      const v=coarse[t];
      if(v>=0){const c=viridis(1-v/c99); r=c[0]*0.75;g=c[1]*0.75;b=c[2]*0.75;}
      else {r=60;g=60;b=66;}
    } else {r=232;g=231;b=226;}
    img.data[pi]=r;img.data[pi+1]=g;img.data[pi+2]=b;img.data[pi+3]=255;
  }
  off.getContext('2d').putImageData(img,0,0);
  draw();
}

function scr(ix,iy){ return [ix*view.s+view.tx, (meta.h-1-iy)*view.s+view.ty]; }

function draw(){
  cv.width=window.innerWidth; cv.height=window.innerHeight-document.getElementById('bar').offsetHeight;
  ctx.fillStyle='#101114'; ctx.fillRect(0,0,cv.width,cv.height);
  ctx.imageSmoothingEnabled=false;
  ctx.drawImage(off, view.tx, view.ty, meta.w*view.s, meta.h*view.s);
  document.getElementById('stage').textContent='場: '+stage;
  if(path){ctx.strokeStyle='#ff5252';ctx.lineWidth=Math.max(1.5,view.s*0.6);ctx.beginPath();
    path.forEach((p,i)=>{const [x,y]=scr(p[0],p[1]); i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();}
  const mark=(c,txt,fill,ink)=>{const [x,y]=scr(c[0]+0.5,c[1]-0.5);
    ctx.fillStyle=fill;ctx.beginPath();ctx.arc(x,y,7,0,7);ctx.fill();
    ctx.fillStyle=ink;ctx.font='bold 10px monospace';ctx.fillText(txt,x-3,y+3.5);};
  if(goal) mark(goal,'G','#35c08d','#0b0b0b');
  if(start) mark(start,'S','#ff5252','#fff');
}

function cellOf(ev){
  const r=cv.getBoundingClientRect();
  const ix=Math.floor((ev.clientX-r.left-view.tx)/view.s);
  const iy=meta.h-1-Math.floor((ev.clientY-r.top-view.ty)/view.s);
  return [ix,iy];
}

const M=()=>document.getElementById('msg');
function applyCoarse(j){
  goal=[j.gx,j.gy]; coarse=j.coarse; field=null; path=null; stage='粗 (タイル粒度)';
  document.getElementById('t_dij').textContent=`Dijkstra: ${j.ms} ms`;
  document.getElementById('t_field').textContent='V̂ 上界: ...';
  document.getElementById('t_exact').textContent='厳密化: –';
  document.getElementById('ex').disabled=true;
  renderRaster();
}
async function snapField(){
  const buf=await (await fetch('/field')).arrayBuffer();
  if(buf.byteLength<100)return;
  field=new Float32Array(buf).subarray(1);
}
let pumpToken=0;
async function pumpField(g){
  const my=++pumpToken;
  let k=0, tsum=0;
  M().textContent='粗い勾配を表示中 — ゴールに近いタイルから V̂ を復元中...';
  while(true){
    if(my!==pumpToken)return;
    const j=await (await fetch(`/field_step?gen=${g}`)).json();
    if(j.err||my!==pumpToken)return;
    tsum+=j.ms;
    document.getElementById('t_field').textContent=
      `V̂ 上界: ${Math.round(100*j.done/j.total)}% (${(tsum/1000).toFixed(1)} s)`;
    if(++k%3===0||j.finished){
      await snapField();
      if(my!==pumpToken)return;
      stage=j.finished?'V̂ (上界)':`V̂ 復元中 ${j.done}/${j.total}`;
      renderRaster();
    }
    if(j.finished){
      document.getElementById('ex').disabled=false;
      M().textContent='V̂ (上界) 完了 — 「厳密化」ボタンで固定点へ。左クリックでスタート設置 (復元中でも可)';
      if(start){ setStart(start[0], start[1]); }
      return;
    }
  }
}
async function setGoal(ix,iy){
  M().textContent='Dijkstra...';
  const j=await (await fetch(`/goal?x=${ix}&y=${iy}`)).json();
  if(j.err){M().textContent=j.err;return;}
  start=null;
  applyCoarse(j);
  pumpField(j.gen);
}
async function doExact(){
  if(!field){M().textContent='先にゴールを置いてください';return;}
  const b=document.getElementById('ex'); b.disabled=true;
  M().textContent='厳密化中... (V̂ 温間の frontier exactify)';
  const buf=await (await fetch('/exact')).arrayBuffer();
  if(buf.byteLength<100){M().textContent='exactify error';b.disabled=false;return;}
  const f32=new Float32Array(buf);
  document.getElementById('t_exact').textContent=`厳密化: ${(f32[0]/1000).toFixed(1)} s`;
  field=f32.subarray(1); stage='厳密 (収束固定点)';
  M().textContent='厳密場を表示中';
  renderRaster();
  if(start){ setStart(start[0], start[1]); }
}
async function setTile(t){
  M().textContent=`tile ${t} に切替中...`;
  const j=await (await fetch(`/tile?t=${t}`)).json();
  if(j.err){M().textContent=j.err;return;}
  meta.tile=j.tile; meta.tnx=j.tnx; meta.tny=j.tny;
  document.getElementById('t_exact').textContent='厳密化: –';
  document.getElementById('ex').disabled=true;
  if(j.coarse){ applyCoarse(j); pumpField(j.gen); }
  else { coarse=null; field=null; path=null; stage='なし';
         M().textContent=`tile ${t} — 右クリックでゴールを置いてください`; renderRaster(); }
}

async function setStart(ix,iy){
  if(!field){document.getElementById('msg').textContent='先に右クリックでゴールを置いてください';return;}
  const th=document.getElementById('th').value;
  const j=await (await fetch(`/path?x=${ix}&y=${iy}&th=${th}`)).json();
  if(j.err){document.getElementById('msg').textContent=j.err;return;}
  start=[ix,iy]; path=j.pts;
  document.getElementById('t_path').textContent=`rollout: ${j.ms} ms`;
  document.getElementById('msg').textContent=(j.reached?'ゴール到達':'経路: '+j.status)+(j.exact?' (厳密場)':' (V̂ 上)');
  draw();
}

// ── パン / ズーム / クリック判別 ──
let drag=null;
cv.addEventListener('mousedown',ev=>{if(ev.button===0)drag={x:ev.clientX,y:ev.clientY,tx:view.tx,ty:view.ty,moved:0};});
window.addEventListener('mousemove',ev=>{if(!drag)return;
  const dx=ev.clientX-drag.x, dy=ev.clientY-drag.y;
  drag.moved=Math.max(drag.moved,Math.abs(dx)+Math.abs(dy));
  view.tx=drag.tx+dx; view.ty=drag.ty+dy; draw();});
window.addEventListener('mouseup',ev=>{
  if(drag && drag.moved<5 && ev.button===0){const [x,y]=cellOf(ev); setStart(x,y);}
  drag=null;});
cv.addEventListener('contextmenu',ev=>{ev.preventDefault();const [x,y]=cellOf(ev);setGoal(x,y);});
cv.addEventListener('wheel',ev=>{ev.preventDefault();
  const r=cv.getBoundingClientRect();
  const mx=ev.clientX-r.left, my=ev.clientY-r.top;
  const k=ev.deltaY<0?1.2:1/1.2;
  const ns=Math.max(0.1,Math.min(40,view.s*k));
  view.tx=mx-(mx-view.tx)*(ns/view.s);
  view.ty=my-(my-view.ty)*(ns/view.s);
  view.s=ns; draw();},{passive:false});
window.addEventListener('resize',draw);

document.getElementById('ex').addEventListener('click',doExact);
(async()=>{
  meta=await (await fetch('/meta')).json();
  const ts=document.getElementById('tile');
  meta.tiles.forEach(t=>{const o=document.createElement('option');o.textContent=t;o.selected=(t===meta.tile);ts.appendChild(o);});
  ts.addEventListener('change',()=>setTile(parseInt(ts.value)));
  mapbits=new Uint8Array(await (await fetch('/map')).arrayBuffer());
  const availH=window.innerHeight-document.getElementById('bar').offsetHeight;
  view.s=Math.min(window.innerWidth/meta.w, availH/meta.h)*0.97;
  view.tx=(window.innerWidth-meta.w*view.s)/2;
  view.ty=(availH-meta.h*view.s)/2;
  document.getElementById('msg').textContent='右クリックでゴールを置いてください';
  renderRaster();
})();
</script></body></html>
"#;
