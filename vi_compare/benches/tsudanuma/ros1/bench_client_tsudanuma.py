#!/usr/bin/env python3
"""ROS1 津田沼ベンチ・タイミング専用クライアント。

bench_client.py と同じ収束検知 (feedback delta) を使うが、巨大マップ
(1963x1334x60) では /value・/policy gridmap の取得が 2.5GB 級になり非現実的
なので **gridmap は取得しない**。goal 送信→収束検知までの wall-clock と
sweep 数だけを記録する。これは bench_map (vi_rs) の solve() 計測と対をなす
本家側の VI 時間。

usage: bench_client_tsudanuma.py GOAL_X GOAL_Y GOAL_YAW_DEG DELTA_THR MAX_SWEEPS TIMEOUT_SEC THREAD_NUM OUT_PREFIX
  DELTA_THR: 本家 _delta は (max_delta>>18) の整数秒。0 で「1秒以上動くセルが無く
             なったら収束」= 本家が宣言する収束点。
"""
import sys, json, time, os
import rospy, actionlib
import numpy as np
from value_iteration.msg import ViAction, ViGoal
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


class Bench:
    def __init__(self, thr, max_sweeps, timeout):
        self.thr = thr
        self.max_sweeps = max_sweeps
        self.timeout = timeout
        self.t0 = None
        self.converged = False
        self.elapsed = None
        self.sweeps = None
        self.last_delta = None
        self.cur_delta = None   # 直近 feedback の max_delta (timeout 時の残差記録用)
        self.cur_sweep = None
        self.client = actionlib.SimpleActionClient('/vi_controller', ViAction)

    def feedback_cb(self, fb):
        deltas = list(fb.deltas.data)
        sweeps = list(fb.current_sweep_times.data)
        if not deltas:
            return
        mx = max(deltas)
        cur = max(sweeps) if sweeps else 0
        el = time.monotonic() - self.t0
        self.cur_delta = mx
        self.cur_sweep = int(cur)
        rospy.loginfo("t=%.1fs sweep=%d max_delta=%g", el, cur, mx)
        if self.elapsed is not None:
            return
        if cur >= 1 and (mx <= self.thr or cur >= self.max_sweeps):
            self.elapsed = el
            self.sweeps = int(cur)
            self.last_delta = mx
            self.converged = bool(mx <= self.thr)
            self.client.cancel_goal()

    def run(self, gx, gy, gyaw_deg):
        rospy.loginfo("waiting for action server /vi_controller ...")
        if not self.client.wait_for_server(rospy.Duration(180)):
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
            rospy.sleep(0.05)
        if self.elapsed is None:  # hard timeout
            self.elapsed = time.monotonic() - self.t0
            self.sweeps = self.cur_sweep if self.cur_sweep else -1
            self.converged = False
            self.last_delta = self.cur_delta   # 停止時点の残差 (生 fixed-point)
            self.client.cancel_goal()
        self.client.wait_for_result(rospy.Duration(30))


def main():
    gx, gy, gyaw = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    thr = float(sys.argv[4])
    max_sweeps = int(sys.argv[5])
    timeout = float(sys.argv[6])
    thread_num = int(sys.argv[7])
    out_prefix = sys.argv[8]

    rospy.init_node('vi_bench_client_tsudanuma')
    b = Bench(thr, max_sweeps, timeout)
    b.run(gx, gy, gyaw)

    os.makedirs(os.path.dirname(out_prefix) or '.', exist_ok=True)
    timing = dict(elapsed_sec=b.elapsed, sweeps=b.sweeps, converged=b.converged,
                  last_max_delta=b.last_delta, thread_num=thread_num,
                  delta_threshold=thr, goal=[gx, gy, gyaw], side='ros1',
                  map='map_tsudanuma_015 (0.15m, scale3)')
    with open(out_prefix + '.json', 'w') as f:
        json.dump(timing, f, indent=2)
    with open(out_prefix + '.csv', 'w') as f:
        f.write('solver,sweeps,elapsed_sec,converged,thread_num\n')
        f.write('ros1_parallel,%s,%.3f,%s,%d\n' % (
            b.sweeps, b.elapsed, 'Y' if b.converged else 'N', thread_num))
    rospy.loginfo("ROS1 tsudanuma bench done: %s", timing)


if __name__ == '__main__':
    main()
