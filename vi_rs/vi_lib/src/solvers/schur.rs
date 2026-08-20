//! タイル通過行列 (ミンプラス Schur 補元) ソルバ — ゴール非依存の前計算で
//! 「任意ゴールの solve」を「ポータルグラフの Dijkstra + タイル復元」に縮める。
//!
//! # 仕組み
//!
//! 地図を `tile` セル角のタイルに割り、タイル領域 (interior + 遷移 reach の halo)
//! の**界面に置いた具体的状態 (x, y, θ) = ポータル**どうしのタイル内最小通過
//! コスト `w(p→q)` を、タイルごとのミニ VI (既存 [`frontier2d`] をタイル寸法の
//! `ValueIterator` に載せる) で前計算する。この行列は静的地図と遷移モデルだけで
//! 決まり**ゴールに依存しない** ([`SchurArtifact`])。
//!
//! クエリ時は (1) ゴールを含むタイルだけ実ゴールを釘付けしたミニ VI →
//! そのタイルのポータルの `d(p→goal)`、(2) ポータルグラフ上の Dijkstra →
//! 全ポータルの `D(p)`、(3) 各タイルはポータルを `total_cost = D(p)` で final
//! 釘付けしたミニ VI で復元、の 3 段。
//!
//! # 上界性 (証明スケッチ)
//!
//! 遷移確率は正確に `2^18 = PROB_BASE` に和が合うので、打ち切り Bellman 作用素は
//! 終端定数シフトと可換: `floor((X + D·2^18)/2^18) = floor(X/2^18) + D`。よって
//! `w(p→q) + D(q)` は「タイル内方策で q に到達 → 以降 D(q)」という**実現可能な
//! 連結方策の打ち切り値そのもの**であり、復元された場 V̂ は方策値の min = v* の
//! 上界。さらに単調作用素の下降は最大不動点に収束するので、V̂ ≥ ν (Reference の
//! MAX_COST 起点固定点) なら V̂ 起点の掃引 (exactify) は **ν に bit-exact** に
//! 着地する。例外は `action_cost_raw` の wrapping_add がオーバーフローする場所
//! (MAX_COST 隣接に大きな確率質量が乗る行動) だけで、そこは単調性が壊れる —
//! conformance の UpperBound 全点アサートと bench の mismatch 計数が経験的ゲート。
//!
//! # 正直な効果範囲
//!
//! 全場実体化はタイルごとのミニ VI の合計 ≈ frontier2d 1 回分で、漸近では
//! 勝たない。勝つのは (1) ポータル Dijkstra が ms 級 (ゴール切替の初動)、
//! (2) ロボット周辺タイルだけの遅延復元、(3) タイル間の完全並列性 (前計算は
//! sweep 順に依存しない — FPGA の複数 CU にそのまま割れる)。
//!
//! `local_penalty` 入りの場・解きかけの場では前計算が上界を保証できないので、
//! [`schur_solve_observed`] は素の [`frontier2d`] へフォールバックする
//! (resweep 挙動はそれで frontier2d と同一 = `caps().resweep = true`)。

use std::collections::{BinaryHeap, HashMap};
use std::io::{self, Read, Write};
use std::path::Path;

use crate::params::{MAX_COST, PROB_BASE};
use crate::solvers::observe::{BoundaryPacer, InPlaceProbe, SolveFlow, SolveObserver, SolveOutcome};
use crate::solvers::{displacement, frontier2d::frontier2d_solve_observed, Bitboard2D, REACH_THRESH};
use crate::state::State;
use crate::value_iterator::{to_index_raw, value_iteration_raw, ValueIterator};

/// 界面 1 本の free 連結区間内でのポータル間隔 [セル]。
// ponytail: 固定間隔 — 地図ごとの調整が要るなら SchurConfig へ昇格。
const PORTAL_SPACING: i32 = 16;
/// タイル GS パス数の上限。wrap なし ([`TILE_INIT`]) の GS は単調降下なので
/// 通常 5〜20 パスで安定する — それを大きく超える遅い尾は早期打ち切り
/// (残る値は下降途中 = 上界なので健全)。
const TILE_MAX_ITER: u32 = 64;
/// ポータル値がこのラウンド数変化しなければタイルソルブを打ち切る。
/// 早期 Stop の値は途中の下降場 = 上界なので、W が上界であるという設計上の
/// 契約 (上界性・exactify の bit-exact 性) はそのまま — 損なうのは W の質だけ。
const STABLE_ROUNDS: u32 = 3;
/// アーティファクトファイルの magic。
const MAGIC: &[u8; 8] = b"VISCHUR1";

/// wrap ゴミの正規化: `action_cost_raw` は MAX_COST 隣接に確率質量が乗ると u64
/// を折り返し、「未到達なのに有限」のゴミ値 (~5e13) を作る。タイル内で q から
/// 壁に隔てられた区画はこのゴミが毎ラウンド振動し、(1) 安定判定が永遠に成立
/// せず TILE_MAX_ITER まで暴走する (scale 依存で実測: scale 2 の house で
/// 1 ソルブ 35 分)、(2) W や V̂ に「偽の有限エッジ」として漏れる。実コストは
/// REACH_THRESH (1e6 秒) を桁で下回るので、それ以上は未到達として扱う。
/// タイルソルブの「未到達」初期値。MAX_COST だと `action_cost_raw` の
/// `wrapping_add` が未到達質量混じりの Q を折り返し、実値**未満**のゴミが
/// W へ漏れたりポータル値が永遠にフラップしたりする (t32 で 1 ソルブが
/// 10^4 パス空転する実測)。REACH_THRESH (1e6 秒) なら
/// (V+pen)·Σprob ≤ ~7e16 で折り返さず、タイル内の演算は真のミンプラス =
/// 単調降下になる。未到達セルは pen ぶんクリープ上昇するが、上界方向で
/// 無害、読み出しは [`clamp_unreached`] (≥ REACH_THRESH → MAX) が弾く。
const TILE_INIT: u64 = REACH_THRESH;

#[inline]
fn clamp_unreached(v: u64) -> u64 {
    if v >= REACH_THRESH {
        MAX_COST
    } else {
        v
    }
}

/// 前計算の設定。
#[derive(Clone, Copy, Debug)]
pub struct SchurConfig {
    /// タイル一辺 [セル]。
    pub tile: i32,
    /// ポータル 1 xy あたりの heading 数 (θ ビンから等間隔に採る)。
    pub portal_thetas: i32,
}

impl Default for SchurConfig {
    fn default() -> Self {
        Self { tile: 32, portal_thetas: 8 }
    }
}

/// ポータル = 地図グローバルセル座標の具体的状態。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Portal {
    pub ix: i32,
    pub iy: i32,
    pub it: i32,
}

/// 1 タイルぶんの通過行列。`w[i * portals.len() + j]` = `portals[i]` から
/// `portals[j]` へ**このタイル領域内だけ**を通る最小コスト (不能は MAX_COST)。
#[derive(Clone, Debug, PartialEq)]
pub struct TileEdges {
    /// [`SchurArtifact::portals`] への索引。
    pub portals: Vec<u32>,
    pub w: Vec<u64>,
}

/// ゴール非依存の前計算成果物: ポータル配置 + タイルごとの通過行列。
#[derive(Clone, Debug, PartialEq)]
pub struct SchurArtifact {
    pub tile: i32,
    pub halo: i32,
    pub nx: i32,
    pub ny: i32,
    pub nt: i32,
    pub tnx: i32,
    pub tny: i32,
    pub portals: Vec<Portal>,
    /// `tiles[ty * tnx + tx]`。
    pub tiles: Vec<TileEdges>,
}

impl SchurArtifact {
    /// タイル領域 (interior + halo) の左下グローバルセル。
    #[inline]
    fn domain_origin(&self, tx: i32, ty: i32) -> (i32, i32) {
        (tx * self.tile - self.halo, ty * self.tile - self.halo)
    }

    /// ミニ VI の一辺 [セル]。
    #[inline]
    fn side(&self) -> i32 {
        self.tile + 2 * self.halo
    }

    /// メモリ上の概算サイズ [byte] (保存形式とほぼ一致)。
    pub fn size_bytes(&self) -> usize {
        let mut n = 8 + 7 * 4 + 4 + self.portals.len() * 12 + 4;
        for t in &self.tiles {
            n += 4 + t.portals.len() * 4 + t.w.len() * 8;
        }
        n
    }

    /// 単純バイナリ (ヘッダ + 配列、LE) で保存する。
    pub fn save(&self, path: &Path) -> io::Result<()> {
        let mut f = io::BufWriter::new(std::fs::File::create(path)?);
        f.write_all(MAGIC)?;
        for v in [self.tile, self.halo, self.nx, self.ny, self.nt, self.tnx, self.tny] {
            f.write_all(&v.to_le_bytes())?;
        }
        f.write_all(&(self.portals.len() as u32).to_le_bytes())?;
        for p in &self.portals {
            for v in [p.ix, p.iy, p.it] {
                f.write_all(&v.to_le_bytes())?;
            }
        }
        f.write_all(&(self.tiles.len() as u32).to_le_bytes())?;
        for t in &self.tiles {
            f.write_all(&(t.portals.len() as u32).to_le_bytes())?;
            for &p in &t.portals {
                f.write_all(&p.to_le_bytes())?;
            }
            for &w in &t.w {
                f.write_all(&w.to_le_bytes())?;
            }
        }
        Ok(())
    }

    pub fn load(path: &Path) -> io::Result<SchurArtifact> {
        let mut f = io::BufReader::new(std::fs::File::open(path)?);
        let mut magic = [0u8; 8];
        f.read_exact(&mut magic)?;
        if &magic != MAGIC {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "not a VISCHUR1 file"));
        }
        let mut i32s = [0i32; 7];
        for v in i32s.iter_mut() {
            *v = read_i32(&mut f)?;
        }
        let [tile, halo, nx, ny, nt, tnx, tny] = i32s;
        let np = read_u32(&mut f)? as usize;
        let mut portals = Vec::with_capacity(np);
        for _ in 0..np {
            portals.push(Portal { ix: read_i32(&mut f)?, iy: read_i32(&mut f)?, it: read_i32(&mut f)? });
        }
        let ntl = read_u32(&mut f)? as usize;
        let mut tiles = Vec::with_capacity(ntl);
        for _ in 0..ntl {
            let n = read_u32(&mut f)? as usize;
            let mut plist = Vec::with_capacity(n);
            for _ in 0..n {
                plist.push(read_u32(&mut f)?);
            }
            let mut w = Vec::with_capacity(n * n);
            for _ in 0..n * n {
                w.push(read_u64(&mut f)?);
            }
            tiles.push(TileEdges { portals: plist, w });
        }
        Ok(SchurArtifact { tile, halo, nx, ny, nt, tnx, tny, portals, tiles })
    }
}

fn read_i32(f: &mut impl Read) -> io::Result<i32> {
    let mut b = [0u8; 4];
    f.read_exact(&mut b)?;
    Ok(i32::from_le_bytes(b))
}
fn read_u32(f: &mut impl Read) -> io::Result<u32> {
    let mut b = [0u8; 4];
    f.read_exact(&mut b)?;
    Ok(u32::from_le_bytes(b))
}
fn read_u64(f: &mut impl Read) -> io::Result<u64> {
    let mut b = [0u8; 8];
    f.read_exact(&mut b)?;
    Ok(u64::from_le_bytes(b))
}

// ──────────────────────────────────────────────────────────────────────────────
// タイルミニ VI
// ──────────────────────────────────────────────────────────────────────────────

/// タイル一辺 `side` の使い回し `ValueIterator`。遷移テーブルは親のクローンで、
/// 再構築しない。free/penalty は**親の states からのコピー**なので、
/// `State::from_occupancy` の行跨ぎバグ込みで本体 solve と一致する
/// (vi_planner の `hydrate_states` と同じ理由)。地図外は free=false = 進入不可。
struct TileVi {
    vi: ValueIterator,
    side: i32,
}

impl TileVi {
    fn new(src: &ValueIterator, side: i32) -> Self {
        let mut vi = ValueIterator::new(src.actions.clone(), 1);
        vi.cell_num_x = side;
        vi.cell_num_y = side;
        vi.cell_num_t = src.cell_num_t;
        vi.xy_resolution = src.xy_resolution;
        vi.t_resolution = src.t_resolution;
        let n = (side * side * src.cell_num_t) as usize;
        let mut states = Vec::with_capacity(n);
        for iy in 0..side {
            for ix in 0..side {
                for it in 0..src.cell_num_t {
                    states.push(State {
                        total_cost: TILE_INIT,
                        penalty: PROB_BASE,
                        local_penalty: 0,
                        ix,
                        iy,
                        it,
                        free: false,
                        final_state: false,
                        optimal_action: None,
                    });
                }
            }
        }
        vi.states = states;
        Self { vi, side }
    }

    /// 領域左下 `(x0, y0)` に窓を置き直し、free/penalty を親からコピーする。
    /// 値・final はリセット。
    fn hydrate(&mut self, src: &ValueIterator, x0: i32, y0: i32) {
        let (side, nt) = (self.side, self.vi.cell_num_t);
        for ly in 0..side {
            let gy = y0 + ly;
            for lx in 0..side {
                let gx = x0 + lx;
                let in_map = gx >= 0 && gx < src.cell_num_x && gy >= 0 && gy < src.cell_num_y;
                for it in 0..nt {
                    let s = &mut self.vi.states[to_index_raw(lx, ly, it, side, nt) as usize];
                    s.total_cost = TILE_INIT;
                    s.local_penalty = 0;
                    s.final_state = false;
                    s.optimal_action = None;
                    if in_map {
                        let g = &src.states[to_index_raw(gx, gy, it, src.cell_num_x, nt) as usize];
                        s.free = g.free;
                        s.penalty = g.penalty;
                    } else {
                        s.free = false;
                        s.penalty = PROB_BASE;
                    }
                }
            }
        }
    }

    /// 値・final だけリセット (free/penalty は保持)。同一タイルでの複数ソルブ用。
    fn reset_values(&mut self) {
        for s in &mut self.vi.states {
            s.total_cost = TILE_INIT;
            s.final_state = false;
            s.optimal_action = None;
        }
    }

    /// ローカルセルを `cost` で final 釘付けする。
    fn pin(&mut self, lx: i32, ly: i32, it: i32, cost: u64) {
        let i = to_index_raw(lx, ly, it, self.side, self.vi.cell_num_t) as usize;
        let s = &mut self.vi.states[i];
        s.final_state = true;
        s.total_cost = cost;
        s.optimal_action = None;
    }

    /// 方向交互の Gauss-Seidel 全面パスで解く (compact.rs の `sweep_rect` と
    /// 同じ骨格)。フロンティア方式はタイル内では波が 1 ラウンドに reach セル
    /// しか進まないが、in-place GS は 1 パスで走査方向へタイルを横断するので
    /// 数パスで収束する。停止は「実質変化 (clamp 後の |Δ|) ≤ TOL のパスが
    /// STABLE 回続く」— wrap ゴミの振動は [`clamp_unreached`] が MAX に正規化
    /// して無視し、±LSB の磨きも TOL が吸収する。早期停止で残る値は下降途中 =
    /// 上界なので、W・V̂ の上界性は保たれる (質だけの問題)。
    ///
    /// `watch = Some(ポータル群)` なら判定をそのセルだけで行う (通過行列は
    /// ポータル値しか読まないので interior の磨きを待たない)。
    fn solve_gs(&mut self, watch: Option<&[(i32, i32, i32)]>) -> u64 {
        const TOL: u64 = 2;
        const STABLE: u32 = 2;
        let (side, nt) = (self.side, self.vi.cell_num_t);
        let mut prev_watch: Vec<u64> = watch
            .map(|w| w.iter().map(|&(x, y, t)| clamp_unreached(self.value(x, y, t))).collect())
            .unwrap_or_default();
        let mut stable = 0u32;
        let mut updates = 0u64;
        for pass in 0..TILE_MAX_ITER {
            let fwd = pass & 1 == 0;
            let mut max_real = 0u64;
            for yy in 0..side {
                let ly = if fwd { yy } else { side - 1 - yy };
                for xx in 0..side {
                    let lx = if fwd { xx } else { side - 1 - xx };
                    for it in 0..nt {
                        let idx = to_index_raw(lx, ly, it, side, nt) as usize;
                        let before = clamp_unreached(self.vi.states[idx].total_cost);
                        if value_iteration_raw(
                            &mut self.vi.states,
                            &self.vi.actions,
                            idx,
                            side,
                            side,
                            nt,
                        ) > 0
                        {
                            updates += 1;
                        }
                        let after = clamp_unreached(self.vi.states[idx].total_cost);
                        max_real = max_real.max(before.abs_diff(after));
                    }
                }
            }
            let stable_now = if let Some(w) = watch {
                let mut same = true;
                for (k, &(x, y, t)) in w.iter().enumerate() {
                    let v = clamp_unreached(self.value(x, y, t));
                    if v.abs_diff(prev_watch[k]) > TOL {
                        prev_watch[k] = v;
                        same = false;
                    }
                }
                // 「未到達のまま不変」を安定と誤読しないためのガード: どれかの
                // watch が到達済みになるまでは、タイル自体の収束 (実質 Δ ≤ TOL)
                // 以外で止めない。さらに最低 3 パス (前進+後退+前進 ≈ どの依存
                // 方向も一度は横断) を要求する。これを落とすと、波が届く前に
                // 2 パス「安定」して W が全 MAX = ポータルグラフ断絶になる
                // (実測: V̂ の 94% が未到達)。
                let any_reached = prev_watch.iter().any(|&v| v < REACH_THRESH);
                same && pass >= 2 && (any_reached || max_real <= TOL)
            } else {
                max_real <= TOL
            };
            stable = if stable_now { stable + 1 } else { 0 };
            if stable >= STABLE {
                break;
            }
        }
        updates
    }

    #[inline]
    fn value(&self, lx: i32, ly: i32, it: i32) -> u64 {
        self.vi.states[to_index_raw(lx, ly, it, self.side, self.vi.cell_num_t) as usize].total_cost
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// 前計算 (build)
// ──────────────────────────────────────────────────────────────────────────────

/// ポータル配置 + タイルごとの通過行列を前計算する。`src` は states 構築済み
/// (dense) であること。ゴールは見ない — 成果物はゴール非依存。
pub fn build(src: &ValueIterator, cfg: &SchurConfig) -> SchurArtifact {
    assert!(!src.states.is_empty(), "SchurArtifact::build needs dense states");
    let (nx, ny, nt) = (src.cell_num_x, src.cell_num_y, src.cell_num_t);
    let (mx, my, _mt) = displacement(src);
    let halo = mx.max(my).max(1);
    let tile = cfg.tile.max(1);
    let tnx = ((nx + tile - 1) / tile).max(1);
    let tny = ((ny + tile - 1) / tile).max(1);
    let k_th = cfg.portal_thetas.clamp(1, nt);

    let free_at = |gx: i32, gy: i32| -> bool {
        src.states[to_index_raw(gx, gy, 0, nx, nt) as usize].free
    };

    // ── ポータル配置: 各界面線の free 連結区間ごとに PORTAL_SPACING 間隔で
    //    xy を置き、そこへ k_th 個の heading を立てる。区間被覆が必須なのは
    //    「固定 k/辺」だとドア (短い区間) を取りこぼすため。
    let mut portals: Vec<Portal> = Vec::new();
    let mut portal_id: HashMap<(i32, i32, i32), u32> = HashMap::new();
    let mut tile_portals: Vec<Vec<u32>> = vec![Vec::new(); (tnx * tny) as usize];

    let place_xy = |gx: i32,
                        gy: i32,
                        owners: &[(i32, i32)],
                        portals: &mut Vec<Portal>,
                        portal_id: &mut HashMap<(i32, i32, i32), u32>,
                        tile_portals: &mut [Vec<u32>]| {
        for i in 0..k_th {
            let it = i * nt / k_th;
            let id = *portal_id.entry((gx, gy, it)).or_insert_with(|| {
                portals.push(Portal { ix: gx, iy: gy, it });
                (portals.len() - 1) as u32
            });
            for &(tx, ty) in owners {
                let lst = &mut tile_portals[(ty * tnx + tx) as usize];
                if !lst.contains(&id) {
                    lst.push(id);
                }
            }
        }
    };

    // free 連結区間 [a..=b] に等間隔で置く位置列。
    let run_positions = |a: i32, b: i32| -> Vec<i32> {
        let len = b - a + 1;
        let n = ((len + PORTAL_SPACING - 1) / PORTAL_SPACING).max(1);
        (0..n).map(|i| a + (2 * i + 1) * len / (2 * n)).collect()
    };

    // 縦界面 x = k*tile (タイル (k-1,ty) と (k,ty) の間)。
    for k in 1..tnx {
        let gx = k * tile;
        if gx >= nx {
            continue;
        }
        for ty in 0..tny {
            let (ya, yb) = (ty * tile, ((ty + 1) * tile).min(ny) - 1);
            let mut y = ya;
            while y <= yb {
                if !free_at(gx, y) {
                    y += 1;
                    continue;
                }
                let a = y;
                while y + 1 <= yb && free_at(gx, y + 1) {
                    y += 1;
                }
                for py in run_positions(a, y) {
                    place_xy(gx, py, &[(k - 1, ty), (k, ty)], &mut portals, &mut portal_id, &mut tile_portals);
                }
                y += 1;
            }
        }
    }
    // 横界面 y = k*tile。
    for k in 1..tny {
        let gy = k * tile;
        if gy >= ny {
            continue;
        }
        for tx in 0..tnx {
            let (xa, xb) = (tx * tile, ((tx + 1) * tile).min(nx) - 1);
            let mut x = xa;
            while x <= xb {
                if !free_at(x, gy) {
                    x += 1;
                    continue;
                }
                let a = x;
                while x + 1 <= xb && free_at(x + 1, gy) {
                    x += 1;
                }
                for px in run_positions(a, x) {
                    place_xy(px, gy, &[(tx, k - 1), (tx, k)], &mut portals, &mut portal_id, &mut tile_portals);
                }
                x += 1;
            }
        }
    }

    let art = SchurArtifact {
        tile,
        halo,
        nx,
        ny,
        nt,
        tnx,
        tny,
        portals,
        tiles: vec![TileEdges { portals: Vec::new(), w: Vec::new() }; (tnx * tny) as usize],
    };

    // ── タイルごとの通過行列。タイル間は完全独立なのでスレッド並列
    //    (これが FPGA 複数 CU に割れる、という主張の CPU 版)。
    let nthr = std::thread::available_parallelism().map(|p| p.get()).unwrap_or(1);
    let tids: Vec<i32> = (0..tnx * tny).collect();
    let chunk = tids.len().div_ceil(nthr).max(1);
    let art_ref = &art;
    let tp_ref = &tile_portals;
    let results: Vec<Vec<(i32, TileEdges)>> = std::thread::scope(|scope| {
        let handles: Vec<_> = tids
            .chunks(chunk)
            .map(|band| {
                scope.spawn(move || {
                    let mut tv = TileVi::new(src, art_ref.side());
                    let mut out = Vec::with_capacity(band.len());
                    for &t in band {
                        out.push((t, tile_edges(src, art_ref, tp_ref, &mut tv, t)));
                    }
                    out
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });

    let mut art = art;
    for part in results {
        for (t, e) in part {
            art.tiles[t as usize] = e;
        }
    }
    art
}

/// 1 タイルの通過行列: 各ポータル q を単独釘付けしたミニ VI で列を埋める。
fn tile_edges(
    src: &ValueIterator,
    art: &SchurArtifact,
    tile_portals: &[Vec<u32>],
    tv: &mut TileVi,
    t: i32,
) -> TileEdges {
    let plist = tile_portals[t as usize].clone();
    let n = plist.len();
    if n == 0 {
        return TileEdges { portals: plist, w: Vec::new() };
    }
    let (tx, ty) = (t % art.tnx, t / art.tnx);
    let (x0, y0) = art.domain_origin(tx, ty);
    tv.hydrate(src, x0, y0);
    let local: Vec<(i32, i32, i32)> = plist
        .iter()
        .map(|&pid| {
            let p = art.portals[pid as usize];
            (p.ix - x0, p.iy - y0, p.it)
        })
        .collect();
    let mut w = vec![MAX_COST; n * n];
    for (j, &(qx, qy, qt)) in local.iter().enumerate() {
        tv.reset_values();
        tv.pin(qx, qy, qt, 0);
        tv.solve_gs(Some(&local));
        for (i, &(px, py, pt)) in local.iter().enumerate() {
            w[i * n + j] = clamp_unreached(tv.value(px, py, pt));
        }
    }
    TileEdges { portals: plist, w }
}

// ──────────────────────────────────────────────────────────────────────────────
// クエリ
// ──────────────────────────────────────────────────────────────────────────────

/// `set_goal` 済みの `vi` から final セル (ix,iy,it) を列挙する。
fn collect_finals(vi: &ValueIterator) -> Vec<(i32, i32, i32)> {
    vi.states
        .iter()
        .filter(|s| s.final_state)
        .map(|s| (s.ix, s.iy, s.it))
        .collect()
}

/// セル (gx,gy) を領域に含むタイルの範囲 (tx0..=tx1, ty0..=ty1)。
fn tiles_covering(art: &SchurArtifact, gx: i32, gy: i32) -> (i32, i32, i32, i32) {
    let lo = |g: i32| ((g - art.tile + 1 - art.halo).div_euclid(art.tile)).max(0);
    let hi = |g: i32, tn: i32| ((g + art.halo).div_euclid(art.tile)).min(tn - 1);
    (lo(gx), hi(gx, art.tnx), lo(gy), hi(gy, art.tny))
}

/// ゴールタイルのミニ VI + ポータルグラフ Dijkstra で全ポータルの
/// `D(p) = d(p → goal)` (上界) を求める。`vi` は `set_goal` 済み・未 solve。
pub fn portal_costs(vi: &ValueIterator, art: &SchurArtifact) -> Vec<u64> {
    let finals = collect_finals(vi);
    let mut d = vec![MAX_COST; art.portals.len()];
    if finals.is_empty() {
        return d;
    }

    // ゴールを領域に含むタイル集合。
    let mut goal_tiles = vec![false; art.tiles.len()];
    for &(gx, gy, _) in &finals {
        let (tx0, tx1, ty0, ty1) = tiles_covering(art, gx, gy);
        for ty in ty0..=ty1 {
            for tx in tx0..=tx1 {
                goal_tiles[(ty * art.tnx + tx) as usize] = true;
            }
        }
    }

    // ゴールタイルごとに実 final を釘付けしたミニ VI → ポータルのシード。
    let mut tv = TileVi::new(vi, art.side());
    for t in 0..art.tiles.len() {
        if !goal_tiles[t] || art.tiles[t].portals.is_empty() {
            continue;
        }
        let (tx, ty) = (t as i32 % art.tnx, t as i32 / art.tnx);
        let (x0, y0) = art.domain_origin(tx, ty);
        tv.hydrate(vi, x0, y0);
        let side = art.side();
        let mut pinned = false;
        for &(gx, gy, it) in &finals {
            let (lx, ly) = (gx - x0, gy - y0);
            if lx >= 0 && lx < side && ly >= 0 && ly < side {
                tv.pin(lx, ly, it, 0);
                pinned = true;
            }
        }
        if !pinned {
            continue;
        }
        let watch: Vec<(i32, i32, i32)> = art.tiles[t]
            .portals
            .iter()
            .map(|&pid| {
                let p = art.portals[pid as usize];
                (p.ix - x0, p.iy - y0, p.it)
            })
            .collect();
        tv.solve_gs(Some(&watch));
        for &pid in &art.tiles[t].portals {
            let p = art.portals[pid as usize];
            let v = clamp_unreached(tv.value(p.ix - x0, p.iy - y0, p.it));
            if v < d[pid as usize] {
                d[pid as usize] = v;
            }
        }
    }

    // ポータル → (tile, slot) 索引。
    let mut membership: Vec<Vec<(u32, u32)>> = vec![Vec::new(); art.portals.len()];
    for (t, e) in art.tiles.iter().enumerate() {
        for (slot, &pid) in e.portals.iter().enumerate() {
            membership[pid as usize].push((t as u32, slot as u32));
        }
    }

    // Dijkstra: D(q) = min over タイル内辺 q→p の w(q→p) + D(p)。
    let mut heap: BinaryHeap<std::cmp::Reverse<(u64, u32)>> = BinaryHeap::new();
    for (i, &v) in d.iter().enumerate() {
        if v < MAX_COST {
            heap.push(std::cmp::Reverse((v, i as u32)));
        }
    }
    while let Some(std::cmp::Reverse((dp, p))) = heap.pop() {
        if dp > d[p as usize] {
            continue;
        }
        for &(t, sp) in &membership[p as usize] {
            let e = &art.tiles[t as usize];
            let n = e.portals.len();
            for sq in 0..n {
                let wqp = e.w[sq * n + sp as usize];
                if wqp >= MAX_COST {
                    continue;
                }
                let cand = wqp.saturating_add(dp);
                if cand >= MAX_COST {
                    continue;
                }
                let q = e.portals[sq];
                if cand < d[q as usize] {
                    d[q as usize] = cand;
                    heap.push(std::cmp::Reverse((cand, q)));
                }
            }
        }
    }
    d
}

/// タイル 1 枚を tv 上で解く (hydrate + ポータル/実 final の釘付け + 収束)。
/// 何も釘付けできなければ false (解かない)。
fn solve_tile(
    src: &ValueIterator,
    art: &SchurArtifact,
    d: &[u64],
    finals: &[(i32, i32, i32)],
    tv: &mut TileVi,
    t: i32,
) -> (bool, u64) {
    let (tx, ty) = (t % art.tnx, t / art.tnx);
    let (x0, y0) = art.domain_origin(tx, ty);
    let side = art.side();
    tv.hydrate(src, x0, y0);
    let mut pinned = false;
    for &pid in &art.tiles[t as usize].portals {
        let dv = d[pid as usize];
        if dv >= MAX_COST {
            continue;
        }
        let p = art.portals[pid as usize];
        tv.pin(p.ix - x0, p.iy - y0, p.it, dv);
        pinned = true;
    }
    for &(gx, gy, it) in finals {
        let (lx, ly) = (gx - x0, gy - y0);
        if lx >= 0 && lx < side && ly >= 0 && ly < side {
            tv.pin(lx, ly, it, 0);
            pinned = true;
        }
    }
    if !pinned {
        return (false, 0);
    }
    (true, tv.solve_gs(None))
}

/// 解けた tv から (global 添字, 値, 方策) の差分列を取り出す。このタイルの釘
/// (値はあるが方策なし) と未到達はスキップ。
fn tile_diffs(tv: &TileVi, art: &SchurArtifact, t: i32) -> Vec<(u32, u64, i8)> {
    let (tx, ty) = (t % art.tnx, t / art.tnx);
    let (x0, y0) = art.domain_origin(tx, ty);
    let side = art.side();
    let nt = tv.vi.cell_num_t;
    let (ix0, ix1) = (x0.max(0), (x0 + side).min(art.nx));
    let (iy0, iy1) = (y0.max(0), (y0 + side).min(art.ny));
    let mut out = Vec::new();
    for gy in iy0..iy1 {
        for gx in ix0..ix1 {
            for it in 0..nt {
                let li = to_index_raw(gx - x0, gy - y0, it, side, nt) as usize;
                let ts = &tv.vi.states[li];
                if ts.final_state || ts.total_cost >= REACH_THRESH {
                    continue; // 釘 / 未到達 / wrap ゴミ ([`clamp_unreached`])
                }
                out.push((
                    to_index_raw(gx, gy, it, art.nx, nt) as u32,
                    ts.total_cost,
                    ts.optimal_action.map_or(-1, |a| a as i8),
                ));
            }
        }
    }
    out
}

/// 差分列を min マージで `vi.states` へ適用する。**領域全体** (interior 限定に
/// しない) の各点 min を取ってはじめて、組み上がった場が方策値族全体の min =
/// Bellman 上界 (V ≥ TV) になる — interior だけ書くと継ぎ目で V < TV が生じ、
/// V̂ の上の貪欲 rollout が循環し得る (実測: LoopDetected)。min を達成した
/// タイルの方策を一緒に書くので Q(s, a(s)) ≤ V(s) も保たれる。
fn merge_diffs(vi: &mut ValueIterator, diffs: &[(u32, u64, i8)]) {
    for &(gi, v, a) in diffs {
        let g = &mut vi.states[gi as usize];
        if g.final_state || v >= g.total_cost {
            continue;
        }
        g.total_cost = v;
        g.optimal_action = (a >= 0).then_some(a as usize);
    }
}

/// タイル 1 枚を復元して `vi.states` へ min マージする。戻り値は更新数。
/// ポータルは `D(p)` で、領域内の実 final は 0 で釘付けする。
pub fn materialize_one(
    vi: &mut ValueIterator,
    art: &SchurArtifact,
    d: &[u64],
    finals: &[(i32, i32, i32)],
    tv: &mut TileScratch,
    t: i32,
) -> u64 {
    let (pinned, updates) = solve_tile(vi, art, d, finals, &mut tv.0, t);
    if !pinned {
        return 0;
    }
    let diffs = tile_diffs(&tv.0, art, t);
    merge_diffs(vi, diffs.as_slice());
    updates
}

/// [`materialize_one`] の使い回しタイル VI (外部から中身に触らせないラッパ)。
pub struct TileScratch(TileVi);

impl TileScratch {
    pub fn new(vi: &ValueIterator, art: &SchurArtifact) -> Self {
        Self(TileVi::new(vi, art.side()))
    }
}

/// ポータルセルへ実行動を与える後処理。ポータルは各タイルで final 釘付けされる
/// ため方策を持たずに実体化される — が、**listing していない被覆タイル** (対角
/// 隣接など、領域にポータルを含むがポータル一覧に持たないタイル) の min マージが
/// 実値+実行動を書いていることが多い。その場合は触らない。残り (方策 None) だけ
/// wrap-free の Bellman で埋める:
///
/// `value_iteration_raw` をグローバル場でそのまま呼んではいけない — 未到達
/// (MAX_COST) の後継が混ざる行動の Q が `wrapping_add` で折り返し、実値**未満**の
/// ゴミがポータルに入る。tsudanuma で実測: 上界破れ 24,190 セル、その下流で
/// exactify (下降のみ) が固定点に届かず mismatch 43k。ここでは MAX_COST 級の
/// 後継を持つ行動を「無効」として除外する (和は (REACH+pen)·2^18 ≈ 7e16 で
/// 折り返さない)。実行動が一つも無ければ MAX のまま = exactify が埋める。
fn fix_portal_actions(vi: &mut ValueIterator, art: &SchurArtifact) -> u64 {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let mut updates = 0;
    for p in &art.portals {
        let idx = to_index_raw(p.ix, p.iy, p.it, art.nx, nt) as usize;
        if vi.states[idx].final_state || vi.states[idx].optimal_action.is_some() {
            continue; // 実ゴール / 被覆タイルの実値が既にある
        }
        let (six, siy, sit) = (vi.states[idx].ix, vi.states[idx].iy, vi.states[idx].it);
        let mut best: Option<(u64, usize)> = None;
        'act: for (ai, a) in vi.actions.iter().enumerate() {
            let mut acc: u64 = 0;
            for tran in &a.state_transitions[sit as usize] {
                let tx = six + tran.dix;
                let ty = siy + tran.diy;
                if tx < 0 || tx >= nx || ty < 0 || ty >= ny {
                    continue 'act;
                }
                let tt = (tran.dit + nt) % nt;
                let after = &vi.states[to_index_raw(tx, ty, tt, nx, nt) as usize];
                if !after.free || after.total_cost >= REACH_THRESH {
                    continue 'act; // 未到達の後継混じり = 無効 (wrap を作らない)
                }
                acc += (after.total_cost + after.penalty + after.local_penalty)
                    * tran.prob as u64;
            }
            let q = acc >> crate::params::PROB_BASE_BIT;
            if best.map_or(true, |(b, _)| q < b) {
                best = Some((q, ai));
            }
        }
        if let Some((q, ai)) = best {
            if q < REACH_THRESH {
                let s = &mut vi.states[idx];
                s.total_cost = q.min(s.total_cost);
                s.optimal_action = Some(ai);
                updates += 1;
            }
        }
    }
    updates
}

/// 上界場 V̂ を組み上げる (ゴールタイル → Dijkstra → 全タイル復元 → ポータル
/// 方策修正)。exactify はしない — bench とテストが V̂ 単体を測るための入口。
/// タイルソルブは独立なのでスレッド並列で解き、min マージだけ直列に適用する
/// (マージは可換な各点 min なので順序不問)。
pub fn upper_bound_field(vi: &mut ValueIterator, art: &SchurArtifact) -> u64 {
    let finals = collect_finals(vi);
    let d = portal_costs(vi, art);
    let n_tiles = art.tnx * art.tny;
    let nthr = std::thread::available_parallelism().map(|p| p.get()).unwrap_or(1);
    let tids: Vec<i32> = (0..n_tiles).collect();
    let chunk = tids.len().div_ceil(nthr).max(1);
    let (src, d_ref, finals_ref) = (&*vi, &d, &finals);
    let parts: Vec<(u64, Vec<(u32, u64, i8)>)> = std::thread::scope(|scope| {
        let handles: Vec<_> = tids
            .chunks(chunk)
            .map(|band| {
                scope.spawn(move || {
                    let mut tv = TileVi::new(src, art.side());
                    let mut updates = 0u64;
                    let mut diffs: Vec<(u32, u64, i8)> = Vec::new();
                    for &t in band {
                        let (pinned, u) = solve_tile(src, art, d_ref, finals_ref, &mut tv, t);
                        if pinned {
                            updates += u;
                            diffs.extend(tile_diffs(&tv, art, t));
                        }
                    }
                    (updates, diffs)
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });
    let mut updates = 0u64;
    for (u, diffs) in &parts {
        updates += u;
        merge_diffs(vi, diffs);
    }
    updates + fix_portal_actions(vi, art)
}

/// V̂ 起点の掃き直し (frontier2d と同じループ、初期フロンティアは値を持つ全セル)。
fn exactify(
    vi: &mut ValueIterator,
    mut iters: u32,
    mut updates: u64,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
    pacer: &mut BoundaryPacer,
) -> SolveOutcome {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let (mx, my, _) = displacement(vi);
    let mut frontier = Bitboard2D::new(nx as u32, ny as u32);
    for s in &vi.states {
        if s.total_cost < MAX_COST {
            frontier.set(s.ix as u32, s.iy as u32);
        }
    }
    while frontier.popcount() > 0 && iters < max_iter {
        iters += 1;
        let candidates = frontier.dilate(mx as u32, my as u32);
        let mut next = Bitboard2D::new(nx as u32, ny as u32);
        for (ix, iy) in candidates.enumerate() {
            let mut u = 0u64;
            for it in 0..nt {
                let idx = to_index_raw(ix as i32, iy as i32, it, nx, nt) as usize;
                if value_iteration_raw(&mut vi.states, &vi.actions, idx, nx, ny, nt) > 0 {
                    u += 1;
                }
            }
            if u > 0 {
                updates += u;
                next.set(ix, iy);
            }
        }
        frontier = next;
        if frontier.popcount() > 0 && pacer.due(iters as u64) {
            let mut probe = InPlaceProbe { vi, iters, updates };
            match obs.boundary(&mut probe) {
                SolveFlow::Continue => {}
                SolveFlow::Stop => return SolveOutcome::stopped(iters, updates),
                SolveFlow::Cancel => return SolveOutcome::cancelled(iters, updates),
            }
        }
    }
    SolveOutcome::running(iters, updates, frontier.popcount() == 0)
}

/// 場が処女 (非 final は全部 MAX_COST、local_penalty なし) か。
/// 偽なら前計算の上界保証が立たないので frontier2d へフォールバックする。
fn is_pristine(vi: &ValueIterator) -> bool {
    vi.states
        .iter()
        .all(|s| s.local_penalty == 0 && (s.final_state || s.total_cost == MAX_COST))
}

/// アーティファクト持ち込みの solve: 復元 (タイルごとに boundary) → exactify。
pub fn schur_solve_with(
    vi: &mut ValueIterator,
    art: &SchurArtifact,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
) -> SolveOutcome {
    if !is_pristine(vi) {
        return frontier2d_solve_observed(vi, max_iter, obs);
    }
    let mut pacer = BoundaryPacer::new(obs);
    let updates = upper_bound_field(vi, art);
    // V̂ 組み上げ全体で 1 反復と数える (タイル並列のため途中境界は入れない —
    // 単タイル粒度の観測が要る側は materialize_one を直接使う)。
    let iters: u32 = 1;
    if pacer.due(iters as u64) {
        let mut probe = InPlaceProbe { vi, iters, updates };
        match obs.boundary(&mut probe) {
            SolveFlow::Continue => {}
            SolveFlow::Stop => return SolveOutcome::stopped(iters, updates),
            SolveFlow::Cancel => return SolveOutcome::cancelled(iters, updates),
        }
    }
    exactify(vi, iters, updates, max_iter, obs, &mut pacer)
}

/// enum 経路: アーティファクトをインライン構築して解く。処女でない場 (掃き直し /
/// local_penalty あり) は素の frontier2d に委譲 — resweep 挙動はそれで
/// frontier2d と同一。
pub fn schur_solve_observed(
    vi: &mut ValueIterator,
    max_iter: u32,
    obs: &mut dyn SolveObserver,
    cfg: SchurConfig,
) -> SolveOutcome {
    if !is_pristine(vi) {
        return frontier2d_solve_observed(vi, max_iter, obs);
    }
    let art = build(vi, &cfg);
    schur_solve_with(vi, &art, max_iter, obs)
}

// ──────────────────────────────────────────────────────────────────────────────
// テスト
// ──────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::action::Action;
    use crate::msg::OccupancyGrid;
    use crate::planner::rollout_path_on;
    use crate::solvers::NullObserver;
    use crate::solvers::test_support::{run_reference_to_fixed_point, REACH};

    /// reach 1 の小行動 (fw 0.05 m @ 0.05 m/cell)。8×8 conformance 地図では
    /// 標準行動の reach 6 がタイルを地図ごと呑み込んで合成が自明化するため、
    /// 実タイル合成はこちらで踏む。
    fn small_actions() -> Vec<Action> {
        vec![
            Action::new("fw", 0.05, 0.0, 0),
            Action::new("back", -0.05, 0.0, 1),
            Action::new("right", 0.0, -30.0, 2),
            Action::new("rightfw", 0.05, -30.0, 3),
            Action::new("left", 0.0, 30.0, 4),
            Action::new("leftfw", 0.05, 30.0, 5),
        ]
    }

    /// x=12 の縦壁に y∈{8,9} のドア。tile=6 で 4×3 タイル、ドアが界面線上に
    /// 乗る (区間被覆ポータルの検証)。ゴールは左室。
    fn corridor_vi() -> ValueIterator {
        let (w, h) = (24i32, 18i32);
        let mut occ = vec![0i8; (w * h) as usize];
        for y in 0..h {
            if !(8..10).contains(&y) {
                occ[(y * w + 12) as usize] = 100;
            }
        }
        let map = OccupancyGrid {
            width: w,
            height: h,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data: occ,
        };
        let mut vi = ValueIterator::new(small_actions(), 1);
        vi.set_map_with_occupancy_grid(&map, 12, 0.05, 30.0, 0.12, 45);
        vi.set_goal(0.15, 0.15, 0);
        vi
    }

    fn cfg() -> SchurConfig {
        SchurConfig { tile: 6, portal_thetas: 4 }
    }

    #[test]
    fn composition_is_upper_bound_and_rollout_sound() {
        let mut exact = corridor_vi();
        run_reference_to_fixed_point(&mut exact);
        let mut vi = corridor_vi();
        let art = build(&vi, &cfg());
        assert!(art.tnx >= 2 && art.tny >= 2, "実タイル合成になっていない");
        assert!(!art.portals.is_empty(), "ポータルが置かれていない");
        // ドア (界面 x=12, y∈{8,9}) にポータルがあること — 区間被覆の要。
        assert!(
            art.portals.iter().any(|p| p.ix == 12 && (8..10).contains(&p.iy)),
            "ドアのポータルが無い: {:?}",
            art.portals
        );
        upper_bound_field(&mut vi, &art);
        for i in 0..exact.states.len() {
            assert!(
                vi.states[i].total_cost >= exact.states[i].total_cost,
                "上界破れ @ {i}: {} < {}",
                vi.states[i].total_cost,
                exact.states[i].total_cost
            );
        }
        // 右室 (ドアの向こう) も復元されていること = 合成が働いている。
        let far = exact
            .states
            .iter()
            .filter(|s| s.free && !s.final_state && s.total_cost < REACH && s.ix > 12)
            .max_by_key(|s| s.total_cost)
            .expect("右室に到達可能セルがあるはず");
        let fi = to_index_raw(far.ix, far.iy, far.it, exact.cell_num_x, exact.cell_num_t) as usize;
        assert!(vi.states[fi].total_cost < MAX_COST, "ドア越しの合成値が無い");
        // V̂ の上の rollout はベストエフォート (greedy rollout は収束場でも
        // プラトーで循環し得る — vi_planner の LoopDetected 許容と同じ)。
        // ここでは「タイル跨ぎ合成が使える方策を出す」ことだけを主張する:
        // exact 場で成功する右室開始点のうち、V̂ でも成功するものが存在する。
        // 成功率そのものは schur_bench の計測項目。
        let res = exact.xy_resolution;
        let (mut exact_ok, mut vhat_ok) = (0u32, 0u32);
        for s0 in exact
            .states
            .iter()
            .filter(|s| s.free && !s.final_state && s.total_cost < REACH && s.ix > 12)
        {
            let sx = exact.map_origin_x + (s0.ix as f64 + 0.5) * res;
            let sy = exact.map_origin_y + (s0.iy as f64 + 0.5) * res;
            let yaw = ((s0.it as f64 + 0.5) * exact.t_resolution).to_radians();
            if !rollout_path_on(&exact, sx, sy, yaw, 4000, 2).reached_goal() {
                continue;
            }
            exact_ok += 1;
            if rollout_path_on(&vi, sx, sy, yaw, 4000, 2).reached_goal() {
                vhat_ok += 1;
            }
        }
        assert!(exact_ok > 0, "exact 場で rollout が成功する開始点が右室に無い");
        assert!(
            vhat_ok > 0,
            "V̂ 上の rollout がドア越しで一度も成功しない ({exact_ok} 起点中)"
        );
    }

    #[test]
    fn solve_with_reaches_reference_fixed_point() {
        let mut exact = corridor_vi();
        run_reference_to_fixed_point(&mut exact);
        let mut vi = corridor_vi();
        let art = build(&vi, &cfg());
        let out = schur_solve_with(&mut vi, &art, 100_000, &mut NullObserver);
        assert!(out.converged);
        for i in 0..exact.states.len() {
            if exact.states[i].total_cost < REACH {
                assert_eq!(
                    exact.states[i].total_cost, vi.states[i].total_cost,
                    "値 mismatch @ {i} (ix={},iy={},it={})",
                    exact.states[i].ix, exact.states[i].iy, exact.states[i].it
                );
                assert_eq!(
                    exact.states[i].optimal_action, vi.states[i].optimal_action,
                    "方策 mismatch @ {i}"
                );
            }
        }
    }


    #[test]
    fn artifact_roundtrip() {
        let vi = corridor_vi();
        let art = build(&vi, &cfg());
        let path = std::env::temp_dir().join(format!("vischur_test_{}.bin", std::process::id()));
        art.save(&path).unwrap();
        let loaded = SchurArtifact::load(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(art, loaded);
    }
}
