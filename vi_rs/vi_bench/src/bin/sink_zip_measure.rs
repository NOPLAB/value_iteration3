//! `bench_map --compact-out-dir` が吐いた生 sink（`compact_value.bin` u64 LE /
//! `compact_action.bin` i32 LE）を `CompressedRamSink` の列符号化に通し、圧縮率と
//! **全セル可逆性**を実測する。解き直しはしない（既存の実データをそのまま教師にする）。
//!
//! 使い方: `cargo run --release -p vi_bench --bin sink_zip_measure -- --dir out --nt 60`

use std::fs::File;
use std::io::{BufReader, Read};
use std::path::PathBuf;

use clap::Parser;

use vi_reference::params::MAX_COST;
use vi_reference::solvers::compact_zip::CompressedRamSink;
use vi_reference::solvers::frontier2d_sparse_compact::CompactSink;

#[derive(Parser)]
struct Args {
    /// compact_value.bin / compact_action.bin のあるディレクトリ。
    #[arg(long, default_value = "out")]
    dir: PathBuf,
    /// θ セル数（列長）。
    #[arg(long, default_value_t = 60)]
    nt: usize,
}

fn main() {
    let args = Args::parse();
    let vpath = args.dir.join("compact_value.bin");
    let apath = args.dir.join("compact_action.bin");
    let vlen = std::fs::metadata(&vpath).expect("compact_value.bin").len() as usize;
    let alen = std::fs::metadata(&apath).expect("compact_action.bin").len() as usize;
    assert_eq!(vlen % 8, 0);
    let nstates = vlen / 8;
    assert_eq!(alen, nstates * 4, "action file size mismatch");
    assert_eq!(nstates % args.nt, 0, "nstates が nt の倍数でない");
    let ncols = nstates / args.nt;
    println!("nstates={nstates} ncols={ncols} nt={} raw={} MB", args.nt, (vlen + alen) >> 20);

    let mut vr = BufReader::with_capacity(1 << 20, File::open(&vpath).unwrap());
    let mut ar = BufReader::with_capacity(1 << 20, File::open(&apath).unwrap());

    let mut sink = CompressedRamSink::new(nstates, args.nt);
    let mut vbuf = vec![0u8; args.nt * 8];
    let mut abuf = vec![0u8; args.nt * 4];
    let mut values = vec![0u64; args.nt];
    let mut actions = vec![0i32; args.nt];
    // 全列の生データを退避して後で read() 照合する（157M states × 12 B ≈ 1.9 GB は持たない —
    // 代わりに列ごとに書いた直後に照合する。キャッシュ経由で復号パスも踏む）。
    let mut skipped = 0usize;
    for col in 0..ncols {
        vr.read_exact(&mut vbuf).unwrap();
        ar.read_exact(&mut abuf).unwrap();
        for i in 0..args.nt {
            values[i] = u64::from_le_bytes(vbuf[i * 8..i * 8 + 8].try_into().unwrap());
            actions[i] = i32::from_le_bytes(abuf[i * 4..i * 4 + 4].try_into().unwrap());
        }
        let base = col * args.nt;
        if values.iter().all(|&v| v == MAX_COST) {
            // 未到達列: solver は finalize しない（sink に書かれない）ので計測でも書かない。
            skipped += 1;
        } else {
            sink.write_column(base, &values, &actions);
        }
        // 可逆性照合（書かない列も sentinel が返ることを確認する）。
        for i in 0..args.nt {
            let (v, a) = sink.read(base + i);
            assert_eq!(v, values[i], "value roundtrip mismatch @ col {col} theta {i}");
            let want_a = if values[i] == MAX_COST { -1 } else { actions[i] };
            assert_eq!(a, want_a, "action roundtrip mismatch @ col {col} theta {i}");
        }
        if col % 500_000 == 0 && col > 0 {
            println!("  … {col}/{ncols} cols");
        }
    }

    let written = sink.written_cols();
    let arena = sink.arena_bytes();
    let index = sink.index_bytes();
    let total = arena + index;
    let raw = vlen + alen;
    println!("written_cols={written} (unreached skipped={skipped})");
    println!("arena={:.1} MB  index={:.1} MB  total={:.1} MB", mb(arena), mb(index), mb(total));
    println!(
        "raw={:.1} MB  ratio={:.2}x  bytes/state={:.2}  bytes/written-col={:.1}",
        mb(raw),
        raw as f64 / total as f64,
        total as f64 / nstates as f64,
        if written > 0 { arena as f64 / written as f64 } else { 0.0 },
    );
    println!("lossless: 全 {nstates} セルの read() が生データと一致");
}

fn mb(b: usize) -> f64 {
    b as f64 / (1024.0 * 1024.0)
}
