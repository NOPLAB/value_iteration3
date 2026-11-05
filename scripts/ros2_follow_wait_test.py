#!/usr/bin/env python3
"""Run after ros2-build, with the ROS and vi_ros2_ws environments sourced."""

import os
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile


def main():
    rclpy.init()
    node = rclpy.create_node("follow_wait_regression")
    map_pub = node.create_publisher(
        OccupancyGrid, "map",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, "mcl_pose", 10)
    commands = []
    subscription = node.create_subscription(Twist, "cmd_vel", commands.append, 100)
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    feedback = []

    def spin_until(predicate, timeout=15.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, "ROS condition timed out"
            rclpy.spin_once(node, timeout_sec=0.05)

    def wait_future(future):
        spin_until(future.done)
        return future.result()

    def send_goal():
        feedback.clear()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = 2.5
        goal.pose.pose.position.y = 1.5
        goal.pose.pose.orientation.w = 1.0
        handle = wait_future(client.send_goal_async(
            goal, feedback_callback=lambda msg: feedback.append(msg.feedback),
        ))
        assert handle.accepted
        return handle, handle.get_result_async()

    def stays_stopped(result):
        commands.clear()
        started = time.monotonic()
        # Count stop ticks too: initial goal preparation may consume part of
        # the wall-clock interval before the follow loop starts.
        while time.monotonic() - started < 4.0 or len(commands) < 40:
            assert time.monotonic() - started < 15.0, "Stop commands timed out"
            rclpy.spin_once(node, timeout_sec=0.05)
            assert not result.done(), "Waiting goal terminated without cancellation"
        assert all(
            cmd.linear.x == cmd.linear.y == cmd.linear.z
            == cmd.angular.x == cmd.angular.y == cmd.angular.z == 0.0
            for cmd in commands
        ), "Waiting published a nonzero velocity"

    def cancel(handle, result):
        assert wait_future(handle.cancel_goal_async()).goals_canceling
        assert wait_future(result).status == GoalStatus.STATUS_CANCELED

    def publish_pose(x):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = 1.5
        pose.pose.pose.orientation.w = 1.0
        pose_pub.publish(pose)

    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width = grid.info.height = 32
    grid.info.resolution = 0.1
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (32 * 32)
    map_pub.publish(grid)

    executable = Path(__file__).resolve().parents[1] / (
        "vi_ros2_ws/install/lib/vi_planner/vi_planner"
    )
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen([
            str(executable), "--ros-args",
            "-p", "standalone:=true", "-p", "goal_retry_limit:=0",
            "-p", "vi_threads:=1", "-p", "map_penalty_radius:=0.0",
        ], stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())
        try:
            spin_until(lambda: client.server_is_ready())
            handle, result = send_goal()
            stays_stopped(result)
            cancel(handle, result)
            print("PASS: missing pose waits beyond 3 seconds, stops and cancels", flush=True)

            spin_until(lambda: pose_pub.get_subscription_count() > 0)
            # Outside the map and the 0.2 m policy borrowing radius, greedy
            # must return NoAction. An unreachable in-map cell can still turn.
            publish_pose(-2.0)
            handle, result = send_goal()
            spin_until(lambda: len(feedback) > 0)
            stays_stopped(result)
            assert len(feedback) >= 30, "NoAction must keep reporting progress"
            assert all(f.number_of_recoveries == 0 for f in feedback)
            cancel(handle, result)
            print("PASS: NoAction waits beyond 3 seconds without retries and cancels", flush=True)

            handle, result = send_goal()
            spin_until(lambda: len(feedback) > 0)
            publish_pose(2.5)
            assert wait_future(result).status == GoalStatus.STATUS_SUCCEEDED
            print("PASS: waiting resumes on pose update and reaches the goal", flush=True)
        finally:
            process.terminate()
            process.wait(timeout=10)
            log.seek(0)
            print(log.read(), flush=True)
            client.destroy()
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
