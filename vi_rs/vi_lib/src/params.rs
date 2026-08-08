//! 本家 `ValueIterator.h` 末尾の静的定数を忠実再現。
//!
//! ```cpp
//! const unsigned char resolution_xy_bit_ = 6;
//! const unsigned char resolution_t_bit_  = 6;
//! const unsigned char prob_base_bit_ = resolution_xy_bit_*2 + resolution_t_bit_; // 18
//! const uint64_t prob_base_ = 1<<prob_base_bit_;            // 262144
//! const uint64_t max_cost_  = 1000000000*prob_base_;        // 262_144_000_000_000
//! ```

pub const RESOLUTION_XY_BIT: u32 = 6;
pub const RESOLUTION_T_BIT: u32 = 6;
pub const PROB_BASE_BIT: u32 = RESOLUTION_XY_BIT * 2 + RESOLUTION_T_BIT; // 18
pub const PROB_BASE: u64 = 1u64 << PROB_BASE_BIT; // 262144
pub const MAX_COST: u64 = 1_000_000_000u64 * PROB_BASE; // 262_144_000_000_000

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constants_match_original() {
        assert_eq!(PROB_BASE_BIT, 18);
        assert_eq!(PROB_BASE, 262_144);
        assert_eq!(MAX_COST, 262_144_000_000_000);
    }
}
