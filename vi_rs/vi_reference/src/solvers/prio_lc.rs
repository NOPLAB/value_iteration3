//! (A2) Priority Label-Correcting — 厳密・本家と bit-exact。
//! `priority_solve(label_setting=false)` の薄ラッパ。

use crate::solvers::priority::priority_solve;
use crate::value_iterator::ValueIterator;

pub fn prio_lc_solve(vi: &mut ValueIterator, max_iter: u32) -> (u32, u64, bool) {
    let st = priority_solve(vi, max_iter, false);
    (st.iters, st.updates, st.converged)
}

#[cfg(test)]
mod tests {
    use super::prio_lc_solve;
    use crate::solvers::test_support::parity_standard_maps;

    #[test]
    fn parity_standard_maps_prio_lc() {
        parity_standard_maps(|vi| prio_lc_solve(vi, 3000));
    }
}
