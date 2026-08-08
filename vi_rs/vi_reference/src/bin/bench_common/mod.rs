//! vi_ref_bench / vi_u64_bench 共有ヘルパ (引数パース・.npy 出力・正典アクション・
//! occupancy 読み込み・value/policy 抽出)。`#[path]` で両バイナリに include する。

use std::fs::File;
use std::io::{self, Write};

use vi_reference::params::PROB_BASE;
use vi_reference::{Action, OccupancyGrid, Quaternion, ValueIterator};

pub fn arg<T: std::str::FromStr>(args: &[String], i: usize, name: &str) -> T
where
    <T as std::str::FromStr>::Err: std::fmt::Display,
{
    args.get(i)
        .unwrap_or_else(|| panic!("missing arg {i} ({name})"))
        .parse::<T>()
        .unwrap_or_else(|e| panic!("bad arg {i} ({name}): {e}"))
}

/// 最小の `.npy` ライタ (float64 '<f8', C-order)。numpy が np.load で読める。
pub fn write_npy_f64(path: &str, shape: &[usize], data: &[f64]) -> io::Result<()> {
    let shape_str = shape.iter().map(|d| d.to_string()).collect::<Vec<_>>().join(", ");
    // 1要素タプルでも trailing comma を付ける (numpy 慣例)。
    let dict = format!(
        "{{'descr': '<f8', 'fortran_order': False, 'shape': ({},), }}",
        shape_str
    );
    let prefix = 10usize; // magic(6) + version(2) + header_len(2)
    let mut header = dict;
    let unpadded = prefix + header.len() + 1; // +1 for trailing '\n'
    let pad = (64 - (unpadded % 64)) % 64;
    for _ in 0..pad {
        header.push(' ');
    }
    header.push('\n');
    let hlen = header.len() as u16;
    let mut f = File::create(path)?;
    f.write_all(b"\x93NUMPY")?;
    f.write_all(&[0x01, 0x00])?;
    f.write_all(&hlen.to_le_bytes())?;
    f.write_all(header.as_bytes())?;
    let mut bytes = Vec::with_capacity(data.len() * 8);
    for &v in data {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    f.write_all(&bytes)
}

pub fn default_actions() -> Vec<Action> {
    // vi_compare の正典 6 アクション (本家 launch と ID 順まで一致)。
    vec![
        Action::new("forward", 0.3, 0.0, 0),
        Action::new("back", -0.2, 0.0, 1),
        Action::new("right", 0.0, -20.0, 2),
        Action::new("rightfw", 0.2, -20.0, 3),
        Action::new("left", 0.0, 20.0, 4),
        Action::new("leftfw", 0.2, 20.0, 5),
    ]
}

/// raw i8 occupancy (Python 側 to_occupancy が生成、row-major) を読み込んで
/// OccupancyGrid を組み立てる。
pub fn load_map(
    occ_raw: &str,
    width: i32,
    height: i32,
    resolution: f64,
    origin_x: f64,
    origin_y: f64,
) -> OccupancyGrid {
    let raw = std::fs::read(occ_raw).expect("read occ_raw");
    let n = (width as usize) * (height as usize);
    assert_eq!(raw.len(), n, "occ_raw size {} != width*height {}", raw.len(), n);
    OccupancyGrid {
        width,
        height,
        resolution,
        origin_x,
        origin_y,
        origin_quat: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        data: raw.iter().map(|&b| b as i8).collect(),
    }
}

/// 価値・方策を (H=cell_num_y, W=cell_num_x, theta) C-order で取り出す。
/// ★本家 valueFunctionWriter は total_cost/prob_base を**整数除算**してから float 化する。
pub fn extract_value_policy(vi: &ValueIterator) -> (Vec<f64>, Vec<f64>, [usize; 3]) {
    let (nx, ny, nt) = (vi.cell_num_x, vi.cell_num_y, vi.cell_num_t);
    let mut value = Vec::with_capacity((nx * ny * nt) as usize);
    let mut policy = Vec::with_capacity((nx * ny * nt) as usize);
    for iy in 0..ny {
        for ix in 0..nx {
            for it in 0..nt {
                let s = &vi.states[vi.to_index(ix, iy, it) as usize];
                value.push((s.total_cost / PROB_BASE) as f64);
                policy.push(match s.optimal_action {
                    Some(ai) => vi.actions[ai].id as f64,
                    None => -1.0,
                });
            }
        }
    }
    (value, policy, [ny as usize, nx as usize, nt as usize])
}
