//! `frontier2d_sparse_compact` の確定出力（value/policy）を**可逆圧縮**して RAM に持つ
//! `CompactSink` 実装（`CompressedRamSink`）。`RamSink` の 12 B/state（u64 値 + i32 方策）を
//! 列単位の符号化で数分の一に落とす。依存クレートは増やさない（純 Rust の varint/RLE）。
//!
//! 符号化（列 = 固定 (ix,iy) の全 θ、`nt ≤ 64`）:
//! - **到達マスク** u64 LE 8 B: θ ビットが 1 ⟺ `value != MAX_COST`。未到達 θ は暗黙に
//!   `(MAX_COST, -1)` なので何も格納しない（compact の finalize が書くのは到達列だけ、
//!   という運用と噛み合う）。
//! - **値**: 到達 θ を昇順に並べた列を **2 階差分 + zigzag varint**。θ 方向の値プロファイルは
//!   「V₂D + 回転コスト × 角度差」の区分線形（V 字）にほぼ従うので、1 階差分は ±定数、
//!   2 階差分は折れ目以外でほぼ 0 → varint 1 B に集中する。u16 量子化と違い**厳密に可逆**。
//! - **方策**: 到達 θ 昇順の action を RLE `(run 長 u8, action i8)`。方策は θ 方向に長い
//!   ランを成すので実質数 B。
//!
//! 制約（solver 用途に特化）:
//! - `write_column` は **finalize の列書き（base % nt == 0, len == nt）専用**。任意 orig 範囲の
//!   部分書きには対応しない。
//! - 追記アリーナ + 列索引なので、同一列の**再書き込みは旧ブロブがゴミとして残る**（索引だけ
//!   張り替え）。solver は各列を一度しか finalize しないので問題にならないが、vi_planner の
//!   `commit_window` / tile repair のような書き換えループ用途にはコンパクションを足すまで
//!   使わないこと。

use std::cell::RefCell;

use crate::params::MAX_COST;

use super::frontier2d_sparse_compact::CompactSink;

/// LEB128 varint を追記する。
#[inline]
fn put_varint(out: &mut Vec<u8>, mut v: u64) {
    loop {
        let b = (v & 0x7f) as u8;
        v >>= 7;
        if v == 0 {
            out.push(b);
            break;
        }
        out.push(b | 0x80);
    }
}

/// LEB128 varint を `pos` から読み進める。
#[inline]
fn get_varint(buf: &[u8], pos: &mut usize) -> u64 {
    let mut v = 0u64;
    let mut shift = 0u32;
    loop {
        let b = buf[*pos];
        *pos += 1;
        v |= ((b & 0x7f) as u64) << shift;
        if b & 0x80 == 0 {
            break v;
        }
        shift += 7;
    }
}

#[inline]
fn zigzag(n: i64) -> u64 {
    ((n << 1) ^ (n >> 63)) as u64
}

#[inline]
fn unzigzag(z: u64) -> i64 {
    ((z >> 1) as i64) ^ -((z & 1) as i64)
}

/// 1 列（θ 全周、`values.len() == nt ≤ 64`）を符号化して `out` へ追記する。
/// フォーマットはモジュール doc の通り。テスト/計測用に公開する。
pub fn encode_column(values: &[u64], actions: &[i32], out: &mut Vec<u8>) {
    let nt = values.len();
    debug_assert!(nt <= 64 && actions.len() == nt);
    let mut mask = 0u64;
    for (i, &v) in values.iter().enumerate() {
        if v != MAX_COST {
            mask |= 1u64 << i;
        }
    }
    out.extend_from_slice(&mask.to_le_bytes());
    // 値: 到達 θ 昇順の列を 2 階差分 + zigzag varint（先頭は絶対値、2 番目は 1 階差分に自然に退化）。
    let mut prev: Option<i64> = None;
    let mut prev_d = 0i64;
    for &v in values.iter().filter(|&&v| v != MAX_COST) {
        debug_assert!(v < MAX_COST && v <= i64::MAX as u64);
        let vv = v as i64;
        match prev {
            None => put_varint(out, zigzag(vv)),
            Some(p) => {
                let d = vv - p;
                put_varint(out, zigzag(d - prev_d));
                prev_d = d;
            }
        }
        prev = Some(vv);
    }
    // 方策: 到達 θ 昇順の action を RLE (run u8, action i8)。
    let mut run: Option<(i32, u8)> = None;
    for (i, &a) in actions.iter().enumerate() {
        if values[i] == MAX_COST {
            continue;
        }
        assert!((-128..=127).contains(&a), "action index が i8 に収まらない: {a}");
        run = match run {
            Some((cur, cnt)) if cur == a && cnt < u8::MAX => Some((cur, cnt + 1)),
            Some((cur, cnt)) => {
                out.push(cnt);
                out.push(cur as i8 as u8);
                Some((a, 1))
            }
            None => Some((a, 1)),
        };
    }
    if let Some((cur, cnt)) = run {
        out.push(cnt);
        out.push(cur as i8 as u8);
    }
}

/// `encode_column` の逆変換。`blob` は 1 列分の完全なブロブ。`values`/`actions`（長さ nt）へ
/// 未到達 θ を `(MAX_COST, -1)` として復元する。
pub fn decode_column(blob: &[u8], nt: usize, values: &mut [u64], actions: &mut [i32]) {
    debug_assert!(values.len() == nt && actions.len() == nt);
    let mask = u64::from_le_bytes(blob[..8].try_into().unwrap());
    let mut pos = 8usize;
    values.fill(MAX_COST);
    actions.fill(-1);
    // 値の復元（2 階差分の積分）。
    let mut prev: Option<i64> = None;
    let mut prev_d = 0i64;
    for (it, v) in values.iter_mut().enumerate().take(nt) {
        if (mask >> it) & 1 == 0 {
            continue;
        }
        let z = unzigzag(get_varint(blob, &mut pos));
        let vv = match prev {
            None => z,
            Some(p) => {
                let d = prev_d + z;
                prev_d = d;
                p + d
            }
        };
        prev = Some(vv);
        *v = vv as u64;
    }
    // 方策の復元（RLE 展開）。
    let mut remain = 0u8;
    let mut cur = -1i32;
    for (it, a) in actions.iter_mut().enumerate().take(nt) {
        if (mask >> it) & 1 == 0 {
            continue;
        }
        if remain == 0 {
            remain = blob[pos];
            cur = blob[pos + 1] as i8 as i32;
            pos += 2;
        }
        *a = cur;
        remain -= 1;
    }
}

/// 追記アリーナ + 列索引の可逆圧縮 RAM sink。未書き込み列の read は `(MAX_COST, -1)`。
pub struct CompressedRamSink {
    nt: usize,
    /// 符号化済み列ブロブの追記アリーナ。
    arena: Vec<u8>,
    /// 列 → `(offset << 24) | len`。0 = 未書き込み（実ブロブは mask 8 B 以上なので len ≥ 8、
    /// entry 0 と衝突しない）。offset は 40 bit（1 TB）、len は 24 bit（16 MB/列）まで。
    cols: Vec<u64>,
    /// 直近に復号した列のキャッシュ `(col, values, actions)`。`read` は orig 昇順（同一列の
    /// θ 連続）で呼ばれることが多いので 1 列で十分効く。`col == usize::MAX` は空。
    cache: RefCell<(usize, Vec<u64>, Vec<i32>)>,
}

impl CompressedRamSink {
    pub fn new(nstates: usize, nt: usize) -> Self {
        assert!(nt >= 1 && nt <= 64, "θ マスクは u64 前提 (nt={nt})");
        assert!(nstates % nt == 0, "nstates ({nstates}) は nt ({nt}) の倍数のはず");
        Self {
            nt,
            arena: Vec::new(),
            cols: vec![0u64; nstates / nt],
            cache: RefCell::new((usize::MAX, vec![MAX_COST; nt], vec![-1; nt])),
        }
    }

    /// 符号化済みアリーナのバイト数（圧縮効果の観測用）。
    pub fn arena_bytes(&self) -> usize {
        self.arena.len()
    }

    /// 列索引のバイト数。
    pub fn index_bytes(&self) -> usize {
        self.cols.len() * 8
    }

    /// 同じ内容を `RamSink`（u64 + i32）で持った場合のバイト数（比較基準）。
    pub fn raw_bytes(&self) -> usize {
        self.cols.len() * self.nt * 12
    }

    /// 書き込み済み列数。
    pub fn written_cols(&self) -> usize {
        self.cols.iter().filter(|&&e| e != 0).count()
    }
}

impl CompactSink for CompressedRamSink {
    fn write_column(&mut self, base: usize, values: &[u64], actions: &[i32]) {
        assert!(
            base % self.nt == 0 && values.len() == self.nt && actions.len() == self.nt,
            "CompressedRamSink は列単位書き専用 (base={base}, len={}, nt={})",
            values.len(),
            self.nt
        );
        let col = base / self.nt;
        let offset = self.arena.len() as u64;
        assert!(offset < (1u64 << 40), "アリーナが 40 bit offset を超えた");
        encode_column(values, actions, &mut self.arena);
        let len = (self.arena.len() as u64 - offset) as u64;
        assert!(len < (1u64 << 24));
        self.cols[col] = (offset << 24) | len;
        // 同列のキャッシュは古くなるので破棄する。
        let mut c = self.cache.borrow_mut();
        if c.0 == col {
            c.0 = usize::MAX;
        }
    }

    fn read(&self, orig: usize) -> (u64, i32) {
        let col = orig / self.nt;
        let it = orig % self.nt;
        let entry = self.cols[col];
        if entry == 0 {
            return (MAX_COST, -1);
        }
        let mut c = self.cache.borrow_mut();
        if c.0 != col {
            let offset = (entry >> 24) as usize;
            let len = (entry & 0xFF_FFFF) as usize;
            let (_, values, actions) = &mut *c;
            decode_column(&self.arena[offset..offset + len], self.nt, values, actions);
            c.0 = col;
        }
        (c.1[it], c.2[it])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solvers::frontier2d_sparse_compact::solve_compact_mapped;
    use std::sync::atomic::AtomicBool;

    /// 符号化の往復が恒等であること。到達穴・全未到達・ゴール値 0・巨大値・方策ランを網羅。
    #[test]
    fn roundtrip_column_patterns() {
        let nt = 60usize;
        let mk = |f: &dyn Fn(usize) -> (u64, i32)| -> (Vec<u64>, Vec<i32>) {
            (0..nt).map(f).unzip()
        };
        let cases: Vec<(Vec<u64>, Vec<i32>)> = vec![
            // V 字（区分線形、2 階差分ほぼ 0）。
            mk(&|i| {
                let d = (i as i64 - 23).unsigned_abs();
                (1_000_000_000 + d * 262_144, ((i / 7) % 6) as i32)
            }),
            // 到達穴あり（偶数 θ のみ到達）。
            mk(&|i| {
                if i % 2 == 0 {
                    (5_000_000_000 + (i as u64) * 3, 2)
                } else {
                    (MAX_COST, -1)
                }
            }),
            // 全未到達。
            mk(&|_| (MAX_COST, -1)),
            // ゴール列（値 0、方策 None）と非単調な値の混在。
            mk(&|i| {
                if i < 5 {
                    (0, -1)
                } else {
                    (262_143_999_999_999u64.saturating_sub(i as u64 * 999_999_937), 5)
                }
            }),
            // 1 θ だけ到達。
            mk(&|i| if i == 59 { (42, 0) } else { (MAX_COST, -1) }),
        ];
        for (values, actions) in cases {
            let mut blob = Vec::new();
            encode_column(&values, &actions, &mut blob);
            let mut dv = vec![0u64; nt];
            let mut da = vec![0i32; nt];
            decode_column(&blob, nt, &mut dv, &mut da);
            assert_eq!(dv, values);
            assert_eq!(da, actions);
        }
    }

    /// mapped 経路 × CompressedRamSink が Reference 固定点と全セルで一致（未到達含む）。
    #[test]
    fn parity_mapped_compressed_sink() {
        use crate::msg::OccupancyGrid;
        use crate::solvers::test_support::{actions, make_vi, run_reference_to_fixed_point, REACH};

        let (w, h) = (32i32, 24i32);
        let mut occ = vec![0i8; (w * h) as usize];
        for iy in 4..20 {
            occ[(iy * w + 13) as usize] = 100; // 縦壁（迂回で θ プロファイルを非自明に）。
        }
        let mut a = make_vi(w, h, occ.clone());
        run_reference_to_fixed_point(&mut a);

        let map = OccupancyGrid {
            width: w,
            height: h,
            resolution: 0.05,
            origin_x: 0.0,
            origin_y: 0.0,
            origin_quat: Default::default(),
            data: occ,
        };
        let nstates = (w * h * 60) as usize;
        let mut sink = CompressedRamSink::new(nstates, 60);
        let s = solve_compact_mapped(
            actions(), 1, &map, 60, 0.2, 30.0, 0.3, 15, 0.10, 0.10, 0, 4000, None, &mut sink, 4,
            &AtomicBool::new(false),
        );
        assert!(s.converged, "mapped solver must converge");

        for i in 0..a.states.len() {
            let (v, act) = sink.read(i);
            if a.states[i].total_cost < REACH {
                assert_eq!(v, a.states[i].total_cost, "value mismatch @ state {i}");
                let act_opt = if act < 0 { None } else { Some(act as usize) };
                assert_eq!(act_opt, a.states[i].optimal_action, "policy mismatch @ state {i}");
            } else {
                assert_eq!((v, act), (MAX_COST, -1), "unreached must read sentinel @ state {i}");
            }
        }
        // 圧縮効果のサニティ: アリーナ + 索引が RamSink 相当より十分小さいこと。
        let total = sink.arena_bytes() + sink.index_bytes();
        assert!(
            total * 3 < sink.raw_bytes(),
            "compressed sink should be < 1/3 of raw (arena={}, index={}, raw={})",
            sink.arena_bytes(),
            sink.index_bytes(),
            sink.raw_bytes()
        );
    }

    /// 同一列の再書き込みは索引が最新を指す（旧ブロブはゴミとして残る、が read は正しい）。
    #[test]
    fn rewrite_reads_latest() {
        let nt = 60usize;
        let mut sink = CompressedRamSink::new(nt * 4, nt);
        let v1: Vec<u64> = (0..nt).map(|i| 1000 + i as u64).collect();
        let a1 = vec![1i32; nt];
        sink.write_column(nt, &v1, &a1);
        assert_eq!(sink.read(nt + 3), (1003, 1));
        let v2: Vec<u64> = (0..nt).map(|i| 2000 + i as u64 * 2).collect();
        let a2 = vec![4i32; nt];
        sink.write_column(nt, &v2, &a2);
        assert_eq!(sink.read(nt + 3), (2006, 4));
        assert_eq!(sink.read(0), (MAX_COST, -1), "未書き込み列は sentinel");
    }
}
