//! `schur_demo` — SchurPortal のインタラクティブデモ。
//!
//! 地図とアーティファクトを常駐させ、ブラウザ GUI (canvas) から
//! 右クリック = ゴール / 左クリック = スタートを受ける:
//!
//! 1. ゴール確定 → ポータル Dijkstra (~100-400 ms) → **タイル粒度の粗い勾配を
//!    即時表示** (各タイル = その ポータルの min D)。
//! 2. 裏で全場 V̂ をタイル並列復元 → 細かい勾配 (min over θ) に置き換え。
//! 3. スタート確定 → V̂ 上の greedy rollout → 経路をオーバーレイ。
//!
//! HTTP は std のみ (依存追加なし)。GET だけの最小実装で、状態は
//! Mutex<Demo> に握らせて逐次処理する。
//!
//!   cargo run --release -p vi_bench --bin schur_demo
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
use vi_lib::solvers::schur::{build, portal_costs, upper_bound_field, SchurArtifact, SchurConfig};
use vi_lib::solvers::REACH_THRESH as REACH;
use vi_lib::{OccupancyGrid, Quaternion, State, ValueIterator};

#[derive(Parser)]
#[command(about = "Interactive SchurPortal demo: click start/goal in the browser, see the gradient.")]
struct Args {
    /// Map YAML。既定は tb3_house。
    #[arg(long)]
    map: Option<PathBuf>,
    #[arg(long, default_value_t = 2)]
    scale: usize,
    #[arg(long, default_value_t = 32)]
    tile: i32,
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
    art: SchurArtifact,
    grid: OccupancyGrid,
    goal_radius_m: f64,
    /// 現在のゴール (セル)。None = 未設定。
    goal: Option<(i32, i32)>,
    /// 現在の D (ゴール確定時に計算)。
    d: Option<Vec<u64>>,
    /// 全場 V̂ が組み上がっているか。
    materialized: bool,
}

impl Demo {
    fn cell_of(&self, wx: f64, wy: f64) -> (i32, i32) {
        (
            (((wx - self.grid.origin_x) / self.grid.resolution).floor() as i32)
                .clamp(0, self.grid.width - 1),
            (((wy - self.grid.origin_y) / self.grid.resolution).floor() as i32)
                .clamp(0, self.grid.height - 1),
        )
    }
    fn world_of(&self, ix: i32, iy: i32) -> (f64, f64) {
        (
            self.grid.origin_x + (ix as f64 + 0.5) * self.grid.resolution,
            self.grid.origin_y + (iy as f64 + 0.5) * self.grid.resolution,
        )
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

    let art_path = args.out.join(format!(
        "schur_demo_{}_s{}_t{}_h{}.bin",
        map_path.file_stem().and_then(|s| s.to_str()).unwrap_or("map"),
        args.scale,
        args.tile,
        args.thetas
    ));
    let art = if art_path.exists() {
        eprintln!("loading artifact: {}", art_path.display());
        SchurArtifact::load(&art_path).expect("load artifact")
    } else {
        eprintln!("building artifact (1 回だけ、次回からキャッシュ) ...");
        let t0 = Instant::now();
        let a = build(&vi, &SchurConfig { tile: args.tile, portal_thetas: args.thetas });
        eprintln!("artifact built in {:.1} s ({} portals)", t0.elapsed().as_secs_f64(), a.portals.len());
        a.save(&art_path).expect("save artifact");
        a
    };
    eprintln!(
        "ready: {} portals, artifact {:.2} MB",
        art.portals.len(),
        art.size_bytes() as f64 / 1e6
    );

    let demo = Mutex::new(Demo { vi, art, grid, goal_radius_m, goal: None, d: None, materialized: false });

    let listener = TcpListener::bind(("127.0.0.1", args.port)).expect("bind");
    eprintln!("open http://127.0.0.1:{}/  (右クリック=ゴール, 左クリック=スタート)", args.port);
    for stream in listener.incoming() {
        let Ok(stream) = stream else { continue };
        // 逐次処理: 状態は 1 つ、リクエストはどのみち Mutex で直列になる。
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
                // xy で重複排除 (θ 違いは 1 点に潰す)。
                let mut seen = std::collections::HashSet::new();
                d.art
                    .portals
                    .iter()
                    .filter(|p| seen.insert((p.ix, p.iy)))
                    .map(|p| format!("[{},{}]", p.ix, p.iy))
                    .collect()
            };
            let json = format!(
                "{{\"w\":{},\"h\":{},\"res\":{},\"tile\":{},\"tnx\":{},\"tny\":{},\"portals\":[{}]}}",
                d.grid.width,
                d.grid.height,
                d.grid.resolution,
                d.art.tile,
                d.art.tnx,
                d.art.tny,
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
                snap_to_free(&d.grid.data, d.grid.width, d.grid.height, ix, iy, 64)
            else {
                return respond(&mut stream, "application/json", b"{\"err\":\"no free cell near goal\"}");
            };
            let (wx, wy) = d.world_of(gx, gy);
            let t0 = Instant::now();
            d.vi.set_goal(wx, wy, 90);
            let dd = portal_costs(&d.vi, &d.art);
            let ms = t0.elapsed().as_secs_f64() * 1e3;
            // 粗い勾配: タイルごとに所属ポータルの min D [秒]。-1 = 情報なし。
            let mut coarse = vec![-1.0f64; d.art.tiles.len()];
            for (t, e) in d.art.tiles.iter().enumerate() {
                let m = e.portals.iter().map(|&p| dd[p as usize]).min().unwrap_or(MAX_COST);
                if m < REACH {
                    coarse[t] = m as f64 / PROB_BASE as f64;
                }
            }
            d.d = Some(dd);
            d.goal = Some((gx, gy));
            d.materialized = false;
            let json = format!(
                "{{\"ms\":{ms:.1},\"gx\":{gx},\"gy\":{gy},\"coarse\":[{}]}}",
                coarse
                    .iter()
                    .map(|v| if *v < 0.0 { "-1".to_string() } else { format!("{v:.1}") })
                    .collect::<Vec<_>>()
                    .join(",")
            );
            respond(&mut stream, "application/json", json.as_bytes())
        }
        // 全場 V̂ を組み上げて min-over-θ の f32 場 [秒] を返す (未到達 = Inf)。
        "/field" => {
            let mut d = demo.lock().unwrap();
            if d.goal.is_none() {
                return respond(&mut stream, "application/json", b"{\"err\":\"set goal first\"}");
            }
            let t0 = Instant::now();
            if !d.materialized {
                let Demo { vi, art, .. } = &mut *d;
                upper_bound_field(vi, art);
                d.materialized = true;
            }
            let ms = t0.elapsed().as_secs_f64() * 1e3;
            let (nx, ny, nt) = (d.grid.width, d.grid.height, N_THETA);
            let mut bytes = Vec::with_capacity((nx * ny * 4) as usize + 32);
            bytes.extend_from_slice(&(ms as f32).to_le_bytes());
            for iy in 0..ny {
                for ix in 0..nx {
                    let base = (ix * nt + iy * nt * nx) as usize;
                    let mut m = MAX_COST;
                    for it in 0..nt as usize {
                        m = m.min(d.vi.states[base + it].total_cost);
                    }
                    let v = if m >= REACH { f32::INFINITY } else { m as f32 / PROB_BASE as f32 };
                    bytes.extend_from_slice(&v.to_le_bytes());
                }
            }
            respond(&mut stream, "application/octet-stream", &bytes)
        }
        // スタート確定: V̂ 上の greedy rollout → 世界座標の折れ線。
        "/path" => {
            let d = demo.lock().unwrap();
            if !d.materialized {
                return respond(&mut stream, "application/json", b"{\"err\":\"field not ready\"}");
            }
            let (ix, iy) = (q("x").unwrap_or(0.0) as i32, q("y").unwrap_or(0.0) as i32);
            let th = q("th").unwrap_or(0.0);
            let Some((sx, sy)) =
                snap_to_free(&d.grid.data, d.grid.width, d.grid.height, ix, iy, 64)
            else {
                return respond(&mut stream, "application/json", b"{\"err\":\"no free cell near start\"}");
            };
            let (wx, wy) = d.world_of(sx, sy);
            let t0 = Instant::now();
            let r = rollout_path_on(&d.vi, wx, wy, th.to_radians(), 100_000, 2);
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
                "{{\"ms\":{ms:.1},\"status\":\"{:?}\",\"reached\":{},\"pts\":[{}]}}",
                r.status,
                r.reached_goal(),
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

// ── ブラウザ GUI (自己完結、依存なし) ──────────────────────────────────────

const HTML: &str = r#"<!doctype html><html><head><meta charset="utf-8">
<title>SchurPortal demo</title>
<style>
body{margin:0;background:#17181b;color:#ecebe6;font-family:"IBM Plex Sans JP","Hiragino Sans",sans-serif;font-size:14px}
#bar{display:flex;gap:1.2em;align-items:center;padding:10px 14px;border-bottom:1px solid #33363c;flex-wrap:wrap}
#bar b{color:#35c08d}
.badge{font-family:ui-monospace,monospace;background:#24262b;border-radius:6px;padding:2px 8px}
#wrap{padding:12px;overflow:auto}
canvas{image-rendering:pixelated;border:1px solid #33363c;border-radius:6px;display:block;cursor:crosshair}
select{background:#24262b;color:#ecebe6;border:1px solid #33363c;border-radius:6px;padding:2px 6px}
.hint{color:#a8acb4}
</style></head><body>
<div id="bar">
  <span><b>SchurPortal</b> デモ</span>
  <span class="hint">右クリック = ゴール / 左クリック = スタート</span>
  <label>開始方位 <select id="th"><option>0</option><option>45</option><option selected>90</option><option>135</option><option>180</option><option>225</option><option>270</option><option>315</option></select>°</label>
  <span class="badge" id="t_dij">Dijkstra: –</span>
  <span class="badge" id="t_field">全場 V̂: –</span>
  <span class="badge" id="t_path">rollout: –</span>
  <span id="msg" class="hint"></span>
</div>
<div id="wrap"><canvas id="cv"></canvas></div>
<script>
const cv = document.getElementById('cv'), ctx = cv.getContext('2d');
let meta, mapbits, field = null, coarse = null, goal = null, start = null, path = null;
let SCALE = 4;

// viridis 5 点近似
function viridis(t){
  const s=[[68,1,84],[59,82,139],[33,145,140],[94,201,98],[253,231,37]];
  t=Math.max(0,Math.min(1,t))*4; const i=Math.min(3,Math.floor(t)), f=t-i;
  return s[i].map((a,k)=>Math.round(a+(s[i+1][k]-a)*f));
}

function draw(){
  const {w,h}=meta;
  cv.width=w*SCALE; cv.height=h*SCALE;
  const img=ctx.createImageData(w,h);
  // 正規化: 有限値の p99
  let p99=1;
  if(field){const fin=[]; for(let i=0;i<w*h;i++){const v=field[i]; if(isFinite(v)&&v>0)fin.push(v);}
    fin.sort((a,b)=>a-b); p99=fin[Math.floor(fin.length*0.99)]||1;}
  let c99=1;
  if(coarse){const fin=coarse.filter(v=>v>=0).sort((a,b)=>a-b); c99=fin[Math.floor(fin.length*0.99)]||1;}
  for(let iy=0;iy<h;iy++)for(let ix=0;ix<w;ix++){
    const gi=iy*w+ix, pi=((h-1-iy)*w+ix)*4; // 格子は y 上向き → 画像は下向き
    let r=20,g=20,b=24;
    if(mapbits[gi]===1){r=g=b=8;}
    else if(field){const v=field[gi];
      if(isFinite(v)){const c=viridis(1-v/p99); r=c[0];g=c[1];b=c[2];}
      else {r=45;g=45;b=52;}
    } else if(coarse){
      const t=Math.floor(ix/meta.tile)+Math.floor(iy/meta.tile)*meta.tnx;
      const v=coarse[t];
      if(v>=0){const c=viridis(1-v/c99); r=c[0]*0.75;g=c[1]*0.75;b=c[2]*0.75;}
    } else {r=232;g=231;b=226;}
    img.data[pi]=r;img.data[pi+1]=g;img.data[pi+2]=b;img.data[pi+3]=255;
  }
  // 等倍で描いてから拡大
  const off=document.createElement('canvas'); off.width=w; off.height=h;
  off.getContext('2d').putImageData(img,0,0);
  ctx.imageSmoothingEnabled=false;
  ctx.drawImage(off,0,0,w*SCALE,h*SCALE);
  const P=(x,y)=>[x*SCALE, (h-1-y)*SCALE];
  if(path){ctx.strokeStyle='#ff5252';ctx.lineWidth=2.5;ctx.beginPath();
    path.forEach((p,i)=>{const [x,y]=P(p[0],p[1]); i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();}
  if(goal){const [x,y]=P(goal[0]+0.5,goal[1]-0.5);ctx.fillStyle='#35c08d';
    ctx.beginPath();ctx.arc(x,y,6,0,7);ctx.fill();ctx.fillStyle='#0b0b0b';ctx.font='9px monospace';ctx.fillText('G',x-3,y+3);}
  if(start){const [x,y]=P(start[0]+0.5,start[1]-0.5);ctx.fillStyle='#ff5252';
    ctx.beginPath();ctx.arc(x,y,6,0,7);ctx.fill();ctx.fillStyle='#fff';ctx.font='9px monospace';ctx.fillText('S',x-3,y+3);}
}

function cellOf(ev){
  const r=cv.getBoundingClientRect();
  const ix=Math.floor((ev.clientX-r.left)/SCALE);
  const iy=meta.h-1-Math.floor((ev.clientY-r.top)/SCALE);
  return [ix,iy];
}

async function setGoal(ix,iy){
  document.getElementById('msg').textContent='Dijkstra...';
  const j=await (await fetch(`/goal?x=${ix}&y=${iy}`)).json();
  if(j.err){document.getElementById('msg').textContent=j.err;return;}
  goal=[j.gx,j.gy]; coarse=j.coarse; field=null; path=null; start=null;
  document.getElementById('t_dij').textContent=`Dijkstra: ${j.ms} ms`;
  document.getElementById('t_field').textContent='全場 V̂: ...';
  document.getElementById('msg').textContent='粗い勾配 (タイル粒度) — 全場を復元中...';
  draw();
  const buf=await (await fetch('/field')).arrayBuffer();
  const f32=new Float32Array(buf);
  document.getElementById('t_field').textContent=`全場 V̂: ${f32[0].toFixed(0)} ms`;
  field=f32.subarray(1);
  document.getElementById('msg').textContent='左クリックでスタートを置くと経路が出ます';
  draw();
}

async function setStart(ix,iy){
  if(!field){document.getElementById('msg').textContent='先に右クリックでゴールを置いてください';return;}
  const th=document.getElementById('th').value;
  const j=await (await fetch(`/path?x=${ix}&y=${iy}&th=${th}`)).json();
  if(j.err){document.getElementById('msg').textContent=j.err;return;}
  start=[ix,iy]; path=j.pts;
  document.getElementById('t_path').textContent=`rollout: ${j.ms} ms`;
  document.getElementById('msg').textContent=j.reached?'ゴール到達':'経路: '+j.status+' (V̂ 上の greedy は稀に循環します — exactify 前の既知の性質)';
  draw();
}

cv.addEventListener('contextmenu',ev=>{ev.preventDefault();const [x,y]=cellOf(ev);setGoal(x,y);});
cv.addEventListener('click',ev=>{const [x,y]=cellOf(ev);setStart(x,y);});

(async()=>{
  meta=await (await fetch('/meta')).json();
  mapbits=new Uint8Array(await (await fetch('/map')).arrayBuffer());
  SCALE=Math.max(1,Math.min(6,Math.floor(1200/meta.w)));
  document.getElementById('msg').textContent='右クリックでゴールを置いてください';
  draw();
})();
</script></body></html>
"#;
