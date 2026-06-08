#!/usr/bin/env bash
# vi_compare/ros2/smoke.sh — end-to-end runtime smoke test for vi_node.
#
# Brings up vi_node, publishes a tiny synthetic /map (8x8 free OccupancyGrid),
# sends one Vi action goal, and asserts the action returns finished=True.
#
# Usage (from repo root, inside the vi_ros2_dev:humble container):
#   bash /workspace/vi_compare/ros2/smoke.sh
#
# Or from host:
#   docker run --rm -v "$(pwd)":/workspace -w /workspace \
#     vi_ros2_dev:humble bash /workspace/vi_compare/ros2/smoke.sh
set -euo pipefail

# ── Source ROS and workspace setups ──────────────────────────────────────────
# The ROS setup scripts reference unbound vars (e.g. AMENT_TRACE_SETUP_FILES),
# so relax nounset while sourcing them, then restore it.
set +u
source /opt/ros/humble/setup.sh
source /ros2_rust_ws/install/local_setup.sh
source /workspace/vi_ros2_ws/install/local_setup.sh
set -u

LOG_DIR="/tmp/vi_smoke_$$"
mkdir -p "$LOG_DIR"
NODE_LOG="$LOG_DIR/vi_node.log"
MAP_LOG="$LOG_DIR/map_pub.log"

echo "=== vi_node smoke test ==="
echo "Logs: $LOG_DIR"

# Cleanup function: kill background processes and report logs on failure.
cleanup() {
    local exit_code=$?
    echo ""
    if [ $exit_code -ne 0 ]; then
        echo "=== FAILED (exit $exit_code) ==="
        echo "--- vi_node log ---"
        cat "$NODE_LOG" 2>/dev/null || echo "(empty)"
        echo "--- map publisher log ---"
        cat "$MAP_LOG" 2>/dev/null || echo "(empty)"
    fi
    # Kill all background jobs
    jobs -p | xargs -r kill 2>/dev/null || true
    wait 2>/dev/null || true
    exit $exit_code
}
trap cleanup EXIT

# ── Step 1: Publish /map with transient_local QoS ────────────────────────────
# CRITICAL: use transient_local + reliable to match vi_node's subscription QoS.
# Do NOT use --once: a publisher that exits does not serve the latched sample to
# a late-joining transient_local subscriber. Keep it alive in the background.
#
# Map: 8x8 free grid, resolution=0.05m, origin=(-0.20, -0.20).
#   - All cells are 0 (free/unoccupied).
#   - Map spans x∈[-0.2, 0.2), y∈[-0.2, 0.2).
#   - Goal at (0.0, 0.0) will land near the center (cell ~[3,3] in 0-indexed)
#     well within the default goal_radius=0.3m.
echo "[1/4] Publishing /map (8x8 free grid, transient_local)..."
ros2 topic pub \
    --qos-durability transient_local \
    --qos-reliability reliable \
    /map nav_msgs/msg/OccupancyGrid \
    '{
        header: {frame_id: "map"},
        info: {
            width: 8, height: 8, resolution: 0.05,
            origin: {position: {x: -0.2, y: -0.2}, orientation: {w: 1.0}}
        },
        data: [0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0]
    }' \
    >"$MAP_LOG" 2>&1 &
MAP_PID=$!

# Give the publisher time to start and serve the first message.
echo "    Waiting 3s for /map publisher to be ready..."
sleep 3

# ── Step 2: Launch vi_node ────────────────────────────────────────────────────
# Parameters:
#   solver=reference        — simplest solver, no rayon
#   map_wait_sec=30         — wait up to 30s for /map
#   safety_radius=0.0       — disable safety dilation to avoid masking the goal
#   goal_margin_radius=0.30 — large enough to cover several cells in the 8x8 map
#   goal_margin_theta=180.0 — accept any heading at the goal (full circle, 360°)
echo "[2/4] Launching vi_node..."
ros2 run vi_node vi_node \
    --ros-args \
    -p solver:=reference \
    -p map_wait_sec:=30 \
    -p safety_radius:=0.0 \
    -p goal_margin_radius:=0.30 \
    -p goal_margin_theta:=180.0 \
    >"$NODE_LOG" 2>&1 &
NODE_PID=$!

# Wait for vi_node to spin up and receive the map (allow up to 20s).
echo "    Waiting 10s for vi_node to start and receive the map..."
sleep 10

# Check the node is still running.
if ! kill -0 "$NODE_PID" 2>/dev/null; then
    echo "ERROR: vi_node exited early!"
    exit 1
fi

# ── Step 3: Send Vi action goal ───────────────────────────────────────────────
# Goal pose:
#   position = (0.0, 0.0)  — map center, well within the 8x8 grid
#   orientation.w = 1.0    — yaw = 0 (facing east)
# The goal_margin_theta=180° means all theta bins are eligible goals, so
# make_goal_mask will mark cells near (0,0) across all headings.
RESULT_FILE="$LOG_DIR/action_result.txt"
echo "[3/4] Sending Vi action goal to /vi_controller..."
timeout 120 ros2 action send_goal \
    /vi_controller \
    vi_interfaces/action/Vi \
    '{goal: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}' \
    2>&1 | tee "$RESULT_FILE" || {
    echo "ERROR: ros2 action send_goal timed out or failed!"
    exit 1
}

# ── Step 4: Assert result contains finished=True ─────────────────────────────
echo "[4/4] Checking action result..."
if grep -q "finished: true" "$RESULT_FILE"; then
    echo ""
    echo "=== SMOKE TEST PASSED ==="
    echo "Action returned: finished=true"
    grep "Result\|finished\|status\|succeeded" "$RESULT_FILE" || true
    exit 0
else
    echo ""
    echo "=== SMOKE TEST FAILED ==="
    echo "Did not find 'finished: true' in action result:"
    cat "$RESULT_FILE"
    exit 1
fi
