#!/usr/bin/env bash
set -euo pipefail

# The ROS setup scripts reference unbound vars (e.g. AMENT_TRACE_SETUP_FILES),
# so relax nounset while sourcing them, then restore it.
set +u
. /opt/ros/humble/setup.sh
. /ros2_rust_ws/install/local_setup.sh
set -u

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS="$REPO_ROOT/vi_ros2_ws"
mkdir -p "$WS/src"
ln -sfn "$REPO_ROOT/vi_ros2/vi_interfaces" "$WS/src/vi_interfaces"
ln -sfn "$REPO_ROOT/vi_ros2/vi_node"       "$WS/src/vi_node"

# Run colcon from $REPO_ROOT (not $WS) so the generated cargo config is found,
# and use --merge-install so the linker finds vi_interfaces' C libs:
#
#  1. config discovery: colcon-ros-cargo writes the generated `.cargo/config.toml`
#     (the [patch.crates-io] redirects to the locally-built rclrs / message /
#     vi_interfaces crates) into colcon's current working directory. The package
#     sources are symlinked into $WS/src, and cargo canonicalizes its cwd through
#     those symlinks to the real paths under $REPO_ROOT/vi_ros2 before searching
#     upward for `.cargo/config.toml`. Run from $WS and the config lands in
#     $WS/.cargo, which the real source tree never sees, so the patches go unused
#     and cargo fails to resolve the ROS crates from crates.io. Running from
#     $REPO_ROOT puts it at $REPO_ROOT/.cargo/config.toml, the common ancestor of
#     both the real package sources and the ../../vi_rs/* path deps.
#
#  2. --merge-install: rosidl_runtime_rs's build.rs adds `<prefix>/lib` to the
#     linker search path for each prefix on AMENT_PREFIX_PATH. With the default
#     isolated install each package gets its own prefix and vi_interfaces' C
#     typesupport libs (libvi_interfaces__rosidl_*_c.so) are not found when
#     linking vi_node. A single merged prefix ($WS/install/lib) puts them on the
#     search path (this is also how the Dockerfile builds rclrs).
cd "$REPO_ROOT"
# Build vi_node (Rust) in cargo's *release* profile for a fair speed comparison
# against the ROS1 C++ baseline (built -O2). `-DCMAKE_BUILD_TYPE=Release` only
# affects CMake packages (vi_interfaces); colcon-ros-cargo 0.2.0 builds the Rust
# crate via `cargo ament-build -- ... --target-dir $WS/build/vi_node --quiet
# <cargo-args>` and (unlike base colcon-cargo) does NOT inject a profile, so
# cargo would otherwise default to the `dev` (debug) profile. A debug-profile
# bit-exact Reference VI solver on 384x384x60 is ~10-50x slower and would not
# converge in the benchmark window. `--cargo-args --release` appends `--release`
# to that cargo invocation, so release artifacts land in $WS/build/vi_node/release/.
# It is placed BEFORE --cmake-args so the trailing "$@" still forwards to
# cmake-args.
colcon build --merge-install --packages-select vi_interfaces vi_node \
       --base-paths "$WS/src" \
       --build-base "$WS/build" \
       --install-base "$WS/install" \
       --cargo-args --release \
       --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
