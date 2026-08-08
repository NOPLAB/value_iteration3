#!/usr/bin/env bash
set -euo pipefail

# The ROS setup scripts reference unbound vars (e.g. AMENT_TRACE_SETUP_FILES),
# so relax nounset while sourcing them, then restore it.
if [ -f /opt/ros/humble/setup.sh ]; then
    # Docker (make ros2-build)
    set +u
    . /opt/ros/humble/setup.sh
    . /ros2_rust_ws/install/local_setup.sh
    set -u
else
    # ネイティブ (RoboStack pixi env + ~/ros2_rust_ws)
    . "$(cd "$(dirname "$0")" && pwd)/ros2_native_env.sh"
fi

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS="$REPO_ROOT/vi_ros2_ws"
mkdir -p "$WS/src"
ln -sfn "$REPO_ROOT/vi_rs/vi_planner" "$WS/src/vi_planner"

# Run colcon from $REPO_ROOT (not $WS) so the generated cargo config is found,
# and use --merge-install so the linker finds the message packages' C libs:
#
#  1. config discovery: colcon-ros-cargo writes the generated `.cargo/config.toml`
#     (the [patch.crates-io] redirects to the locally-built rclrs / message
#     crates) into colcon's current working directory. The package sources are
#     symlinked into $WS/src, and cargo canonicalizes its cwd through those
#     symlinks to the real paths under $REPO_ROOT/vi_rs before searching
#     upward for `.cargo/config.toml`. Run from $WS and the config lands in
#     $WS/.cargo, which the real source tree never sees, so the patches go unused
#     and cargo fails to resolve the ROS crates from crates.io. Running from
#     $REPO_ROOT puts it at $REPO_ROOT/.cargo/config.toml, an ancestor of
#     both the real package sources and the ../vi_lib path dep.
#
#  2. --merge-install: rosidl_runtime_rs's build.rs adds `<prefix>/lib` to the
#     linker search path for each prefix on AMENT_PREFIX_PATH (this is also how
#     the Dockerfile builds rclrs / nav2_msgs in /ros2_rust_ws).
cd "$REPO_ROOT"
# Build the Rust node in cargo's *release* profile. colcon-ros-cargo 0.2.0
# builds via `cargo ament-build -- ... <cargo-args>` and does NOT inject a
# profile, so cargo would otherwise default to the `dev` (debug) profile —
# a debug-profile VI solver is ~10-50x slower. `--cargo-args --release` is
# placed BEFORE --cmake-args so the trailing "$@" still forwards to cmake-args.
colcon build --merge-install --packages-select vi_planner \
       --base-paths "$WS/src" \
       --build-base "$WS/build" \
       --install-base "$WS/install" \
       --log-base "$WS/log" \
       --cargo-args --release \
       --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
