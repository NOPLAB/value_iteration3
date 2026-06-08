//! Minimal dependency-free `.npy` writer for Array3<u16>/<i16> (C order).

use std::fs::File;
use std::io::{self, Write};
use ndarray::Array3;

fn write_header(f: &mut File, descr: &str, shape: &[usize]) -> io::Result<()> {
    let shape_str = shape
        .iter()
        .map(|d| d.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    let dict = format!(
        "{{'descr': '{}', 'fortran_order': False, 'shape': ({},), }}",
        descr, shape_str
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
    f.write_all(b"\x93NUMPY")?;
    f.write_all(&[0x01, 0x00])?;
    f.write_all(&hlen.to_le_bytes())?;
    f.write_all(header.as_bytes())?;
    Ok(())
}

pub fn write_u16(path: &str, arr: &Array3<u16>) -> io::Result<()> {
    let std = arr.as_standard_layout();
    let mut f = File::create(path)?;
    write_header(&mut f, "<u2", std.shape())?;
    let mut bytes = Vec::with_capacity(std.len() * 2);
    for &v in std.iter() {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    f.write_all(&bytes)
}

pub fn write_i16(path: &str, arr: &Array3<i16>) -> io::Result<()> {
    let std = arr.as_standard_layout();
    let mut f = File::create(path)?;
    write_header(&mut f, "<i2", std.shape())?;
    let mut bytes = Vec::with_capacity(std.len() * 2);
    for &v in std.iter() {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    f.write_all(&bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndarray::Array3;

    #[test]
    fn header_is_64_aligned_and_data_follows() {
        let a = Array3::<u16>::from_shape_fn((2, 3, 4), |(i, j, k)| (i * 100 + j * 10 + k) as u16);
        let path = std::env::temp_dir().join("vi_npy_test.npy");
        let p = path.to_str().unwrap();
        write_u16(p, &a).unwrap();
        let bytes = std::fs::read(p).unwrap();
        assert_eq!(&bytes[0..6], b"\x93NUMPY");
        let hlen = u16::from_le_bytes([bytes[8], bytes[9]]) as usize;
        assert_eq!((10 + hlen) % 64, 0);
        assert_eq!(bytes.len(), 10 + hlen + 2 * 3 * 4 * 2);
        let off = 10 + hlen;
        assert_eq!(u16::from_le_bytes([bytes[off], bytes[off + 1]]), 0);
    }
}
