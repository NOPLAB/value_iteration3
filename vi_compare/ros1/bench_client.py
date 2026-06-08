#!/usr/bin/env python3
"""ROS1 bench client: send goal, time convergence via feedback deltas,
preempt at convergence, fetch /value & /policy gridmaps, dump npy + timing."""
import sys, json, time, os
import numpy as np
import rospy, actionlib, yaml
from value_iteration.msg import ViAction, ViGoal
from grid_map_msgs.srv import GetGridMap
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

UNREACHABLE = 1e6  # value >= this => unreachable (max_cost_/prob_base_ == 1e9)

class Bench:
    def __init__(self, p):
        self.thr = p['client']['delta_threshold']
        self.max_sweeps = p['client']['max_sweeps']
        self.timeout = p['client']['timeout_sec']
        self.t0 = None
        self.converged = False
        self.elapsed = None
        self.sweeps = None
        self.client = actionlib.SimpleActionClient('/vi_controller', ViAction)

    def feedback_cb(self, fb):
        deltas = list(fb.deltas.data)
        sweeps = list(fb.current_sweep_times.data)
        if not deltas:
            return
        mx = max(deltas)
        cur = max(sweeps) if sweeps else 0
        rospy.loginfo("sweep=%d max_delta=%g", cur, mx)
        if self.converged or self.elapsed is not None:
            return
        if cur >= 1 and (mx <= self.thr or cur >= self.max_sweeps):
            self.elapsed = time.monotonic() - self.t0
            self.sweeps = int(cur)
            self.converged = bool(mx <= self.thr)
            self.client.cancel_goal()

    def run(self, gx, gy, gyaw_deg):
        rospy.loginfo("waiting for action server /vi_controller ...")
        if not self.client.wait_for_server(rospy.Duration(120)):
            raise RuntimeError('vi_controller action server not available')
        goal = ViGoal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        q = quaternion_from_euler(0.0, 0.0, np.deg2rad(gyaw_deg))
        ps.pose.orientation.x, ps.pose.orientation.y = q[0], q[1]
        ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
        goal.goal = ps
        self.t0 = time.monotonic()
        self.client.send_goal(goal, feedback_cb=self.feedback_cb)
        deadline = self.t0 + self.timeout
        while not rospy.is_shutdown() and self.elapsed is None and time.monotonic() < deadline:
            rospy.sleep(0.02)
        if self.elapsed is None:  # hard timeout
            self.elapsed = time.monotonic() - self.t0
            self.sweeps = -1
            self.converged = False
            self.client.cancel_goal()
        self.client.wait_for_result(rospy.Duration(15))

def decode_gridmap(gm):
    """Return ndarray [theta, ix, iy] of the grid_map layers '0'..'N-1'."""
    res = gm.info.resolution
    rows = int(round(gm.info.length_x / res))   # x dimension (== cell_num_x)
    cols = int(round(gm.info.length_y / res))   # y dimension (== cell_num_y)
    idx = {name: i for i, name in enumerate(gm.layers)}
    n_theta = len(gm.layers)
    out = np.full((n_theta, rows, cols), np.nan, dtype=np.float64)
    for t in range(n_theta):
        arr = np.array(gm.data[idx[str(t)]].data, dtype=np.float64)
        # grid_map: Eigen column-major; map.at(Index(ix,iy)) => M[ix, iy]
        out[t] = arr.reshape((rows, cols), order='F')
    return out  # [theta, ix, iy]

def fetch(name):
    rospy.loginfo("waiting for service %s ...", name)
    rospy.wait_for_service(name, timeout=60)
    return rospy.ServiceProxy(name, GetGridMap)().map

def main():
    params_path, out_dir = sys.argv[1], sys.argv[2]
    with open(params_path) as f:
        p = yaml.safe_load(f)
    rospy.init_node('vi_bench_client_ros1')
    b = Bench(p)
    g = p['goal']
    b.run(g['x'], g['y'], g['yaw_deg'])

    value = decode_gridmap(fetch('/value'))    # [theta, ix, iy], step units
    policy = decode_gridmap(fetch('/policy'))  # [theta, ix, iy], action id or -1
    # canonical [iy, ix, theta] to match vi_ros2 (H=y, W=x, theta)
    value = np.transpose(value, (2, 1, 0))
    policy = np.transpose(policy, (2, 1, 0))

    os.makedirs(out_dir, exist_ok=True)
    np.save(os.path.join(out_dir, 'value_ros1.npy'), value.astype(np.float64))
    np.save(os.path.join(out_dir, 'policy_ros1.npy'), policy.astype(np.float64))
    timing = dict(elapsed_sec=b.elapsed, sweeps=b.sweeps, converged=b.converged,
                  thread_num=p['planning']['thread_num'],
                  delta_threshold=b.thr, side='ros1')
    with open(os.path.join(out_dir, 'timing_ros1.json'), 'w') as f:
        json.dump(timing, f, indent=2)
    rospy.loginfo("ROS1 bench done: %s", timing)

if __name__ == '__main__':
    main()
