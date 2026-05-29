//! Black-box integration test. Skipped unless running inside the Docker
//! image built in Task 11 (rclrs is required to link).
//!
//! The test:
//!   1. Spawns `vi_node` with a tiny inline-published map.
//!   2. Sends a Vi.action goal at the map centre.
//!   3. Asserts that feedback contains exactly one element per array.
//!   4. Asserts that the result reports `finished: true` within 10 s.

#![cfg(feature = "integration")]

#[test]
fn vi_node_converges_on_tiny_empty_map() {
    // See `scripts/ros2_test.sh` — this test is gated on the `integration`
    // Cargo feature and executed only when the colcon environment is sourced.
    // Implementation pattern: spawn a separate rclrs client process via
    // `std::process::Command::new("ros2")` with `action send_goal`, parse
    // the YAML reply. Equivalent native rclrs client implementation is
    // acceptable but slower to write.
    //
    // For now this is a SCAFFOLD only — wiring the actual ros2 invocation
    // is left as an open task because it requires the full Docker-side
    // rclrs build to be working.
    panic!("integration test not yet implemented; see comment");
}
