#!/usr/bin/env python3
"""ROS2 bench client: publish /map (house.pgm), send Vi goal, time to result.
value/policy are dumped by vi_node (bench_dump_path)."""
import sys, json, time, os
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from vi_interfaces.action import Vi

def load_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxv = int(f.readline())
        data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape((h, w))
    return w, h, data

def load_map_yaml(pgm_path):
    """Read the sibling <map>.yaml (map_server format) for geometry + thresholds."""
    yaml_path = os.path.splitext(pgm_path)[0] + '.yaml'
    with open(yaml_path) as f:
        m = yaml.safe_load(f)
    origin = m.get('origin', [0.0, 0.0, 0.0])
    return dict(resolution=float(m['resolution']),
                ox=float(origin[0]), oy=float(origin[1]),
                occupied_thresh=float(m.get('occupied_thresh', 0.65)),
                free_thresh=float(m.get('free_thresh', 0.196)),
                negate=int(m.get('negate', 0)))

def to_occupancy(w, h, pgm, meta):
    # map_server semantics (negate=0): occ_prob = (255 - p)/255
    p = pgm.astype(np.float64)
    occ_prob = (p / 255.0) if meta['negate'] else ((255.0 - p) / 255.0)
    occ = np.full((h, w), -1, dtype=np.int8)              # default: unknown
    occ[occ_prob < meta['free_thresh']] = 0               # free
    occ[occ_prob > meta['occupied_thresh']] = 100         # occupied
    # ROS OccupancyGrid is row-major bottom-up (origin bottom-left) -> flip vertically
    occ = np.flipud(occ)
    msg = OccupancyGrid()
    msg.info.resolution = meta['resolution']
    msg.info.width = w
    msg.info.height = h
    msg.info.origin.position.x = meta['ox']
    msg.info.origin.position.y = meta['oy']
    msg.info.origin.orientation.w = 1.0
    msg.data = occ.reshape(-1).tolist()
    return msg

class BenchNode(Node):
    def __init__(self, p, map_msg):
        super().__init__('vi_bench_client_ros2')
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history = QoSHistoryPolicy.KEEP_LAST
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos)
        self.map_pub.publish(map_msg)
        self.ac = ActionClient(self, Vi, 'vi_controller')
        self.p = p
        self.elapsed = None
        self.sweeps = None
        self.converged = None

    def send(self):
        g = self.p['goal']
        if not self.ac.wait_for_server(timeout_sec=120):
            raise RuntimeError('vi_controller action server not available')
        goal = Vi.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(g['x'])
        ps.pose.position.y = float(g['y'])
        yaw = np.deg2rad(g['yaw_deg'])
        ps.pose.orientation.z = float(np.sin(yaw / 2))
        ps.pose.orientation.w = float(np.cos(yaw / 2))
        goal.goal = ps
        t0 = time.monotonic()
        self._last_sweep = 0
        fut = self.ac.send_goal_async(goal, feedback_callback=self._fb)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)
        self.elapsed = time.monotonic() - t0
        self.converged = bool(rfut.result().result.finished)
        self.sweeps = int(self._last_sweep)

    def _fb(self, fb):
        d = fb.feedback.current_sweep_times.data
        if d:
            self._last_sweep = max(d)

def main():
    params_path, map_path, out_dir = sys.argv[1], sys.argv[2], sys.argv[3]
    with open(params_path) as f:
        p = yaml.safe_load(f)
    w, h, pgm = load_pgm(map_path)
    meta = load_map_yaml(map_path)
    map_msg = to_occupancy(w, h, pgm, meta)
    rclpy.init()
    node = BenchNode(p, map_msg)
    node.send()
    os.makedirs(out_dir, exist_ok=True)
    timing = dict(elapsed_sec=node.elapsed, sweeps=node.sweeps,
                  converged=node.converged,
                  thread_num=p['planning']['thread_num'],
                  delta_threshold=p['client']['delta_threshold'], side='ros2')
    with open(os.path.join(out_dir, 'timing_ros2.json'), 'w') as f:
        json.dump(timing, f, indent=2)
    node.get_logger().info(f"ROS2 bench done: {timing}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
