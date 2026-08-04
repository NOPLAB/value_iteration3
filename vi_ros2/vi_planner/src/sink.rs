//! compact (アウトオブコア) 経路の確定出力をディスク mmap に置く `CompactSink` 実装。
//!
//! `vi_reference` は依存を軽く保つため mmap 実装を持たない (`RamSink` のみ) ので、
//! `vi_bench::bench_map` / `vi_global_planner::sink` と同じ実装をここにも置く
//! (クレートが別なので共有できない — 片方を直したら両方直すこと)。value (u64 LE) と
//! policy (i32 LE) を 2 ファイルに分け、finalize 時は列連続で write、
//! ロールアウト時は orig 単位で read する。
//!
//! 用途: 津田沼のような広域地図では確定出力が `nx·ny·nθ × 12 B` になる
//! (0.15 m/cell で 157M states = 1.9 GB)。Pi4 4GB では RAM に置けないのでディスクへ外す。
//! 未書き込み = 到達不能 → `(MAX_COST, -1)`。

use memmap2::MmapMut;
use vi_reference::params::MAX_COST;
use vi_reference::solvers::frontier2d_sparse_compact::CompactSink;

pub struct MmapSink {
    value: MmapMut,  // nstates * 8 bytes
    action: MmapMut, // nstates * 4 bytes
    /// 捨てるときにディレクトリごと消すか (`new_owned` で作ったときだけ Some)。
    /// 先読み (`waypoint_prefetch`) は場を 2 つ同時に生かすので solve ごとに別
    /// ディレクトリを使う。誰も消さないと 1 巡ごとに sink が 1 つ積み上がる。
    owned_dir: Option<std::path::PathBuf>,
}

impl MmapSink {
    pub fn new(dir: &std::path::Path, nstates: usize) -> std::io::Result<Self> {
        Self::open(dir, nstates, None)
    }

    /// `new` と同じだが、**この sink がディレクトリを所有する** — 捨てるときに
    /// `dir` ごと消す。solve のたびに使い捨てのディレクトリを切る呼び出し用。
    pub fn new_owned(dir: &std::path::Path, nstates: usize) -> std::io::Result<Self> {
        Self::open(dir, nstates, Some(dir.to_path_buf()))
    }

    fn open(
        dir: &std::path::Path,
        nstates: usize,
        owned_dir: Option<std::path::PathBuf>,
    ) -> std::io::Result<Self> {
        std::fs::create_dir_all(dir)?;
        let map = |name: &str, bytes: usize| -> std::io::Result<MmapMut> {
            let f = std::fs::OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .truncate(true)
                .open(dir.join(name))?;
            f.set_len(bytes as u64)?;
            unsafe { MmapMut::map_mut(&f) }
        };
        let mut value = map("compact_value.bin", nstates * 8)?;
        let mut action = map("compact_action.bin", nstates * 4)?;
        // 初期化: 未書き込み (到達不能) セルが (MAX_COST, None) と読めるように。
        let le = MAX_COST.to_le_bytes();
        for rec in value.chunks_exact_mut(8) {
            rec.copy_from_slice(&le);
        }
        action.fill(0xFF); // i32 -1 = 全バイト 0xFF。
        Ok(Self { value, action, owned_dir })
    }
}

impl Drop for MmapSink {
    fn drop(&mut self) {
        let Some(dir) = self.owned_dir.as_ref() else { return };
        // マッピングはこの後 (フィールドの drop で) 外れるが、Linux では
        // mmap 中のファイルを unlink してよい (inode は写像が生きている間残る)。
        // Windows では拒否されるので、握り潰して残す — 使い捨てディレクトリを
        // 使うのは実機 (Linux) の先読みだけで、ホストのテストは自分で片付ける。
        let _ = std::fs::remove_dir_all(dir);
    }
}

impl CompactSink for MmapSink {
    fn write_column(&mut self, base: usize, values: &[u64], actions: &[i32]) {
        let vb = &mut self.value[base * 8..(base + values.len()) * 8];
        for (i, &v) in values.iter().enumerate() {
            vb[i * 8..i * 8 + 8].copy_from_slice(&v.to_le_bytes());
        }
        let ab = &mut self.action[base * 4..(base + actions.len()) * 4];
        for (i, &a) in actions.iter().enumerate() {
            ab[i * 4..i * 4 + 4].copy_from_slice(&a.to_le_bytes());
        }
    }

    fn read(&self, orig: usize) -> (u64, i32) {
        let v = u64::from_le_bytes(self.value[orig * 8..orig * 8 + 8].try_into().unwrap());
        let a = i32::from_le_bytes(self.action[orig * 4..orig * 4 + 4].try_into().unwrap());
        (v, a)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unwritten_cells_read_as_unreachable_and_roundtrip() {
        let dir = std::env::temp_dir().join("vi_planner_sink_test");
        let mut s = MmapSink::new(&dir, 8).expect("create sink");
        assert_eq!(s.read(0), (MAX_COST, -1));
        s.write_column(2, &[7, 9], &[3, -1]);
        assert_eq!(s.read(2), (7, 3));
        assert_eq!(s.read(3), (9, -1));
        assert_eq!(s.read(4), (MAX_COST, -1));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
