#!/usr/bin/env python3
"""Report when RRT-Cut first latches a real sub-goal after the configured goal."""

import os
import re
import sys
import time
import xmlrpc.client
from pathlib import Path

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Log


GOAL_FILE = Path(__file__).resolve().parents[1] / "src/user_config/goal_config.yaml"
SUBGOAL_LOG = re.compile(r"Sub-goal \((-?\d+), (-?\d+)\) kept until the robot reaches it")


def wait_for_master():
    uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    print(f"Đang chờ ROS master tại {uri}...", flush=True)
    while True:
        try:
            xmlrpc.client.ServerProxy(uri).getPid("/watch_first_rrt_cut_subgoal")
            return
        except (OSError, xmlrpc.client.Error):
            time.sleep(1)


def main():
    with GOAL_FILE.open(encoding="utf-8") as stream:
        configured_goal = yaml.safe_load(stream)["goal"]
    goal_x, goal_y = float(configured_goal["x"]), float(configured_goal["y"])
    print(f"Chờ goal thật ({goal_x:.3f}, {goal_y:.3f}); bỏ qua goal khởi tạo (0, 0).", flush=True)

    wait_for_master()
    rospy.init_node("watch_first_rrt_cut_subgoal", anonymous=True)
    state = {"goal_time": None}

    def on_goal(msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        if abs(x - goal_x) > 0.05 or abs(y - goal_y) > 0.05:
            return
        planner = rospy.get_param("/move_base/SamplePlanner/planner_name", None)
        if planner != "rrt_cut":
            print(f"Bỏ qua goal: planner đang chạy là {planner!r}, không phải rrt_cut.", flush=True)
            return
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        state["goal_time"] = stamp.to_sec()
        print(f"GOAL thật: ROS time = {state['goal_time']:.3f} s; ({x:.3f}, {y:.3f}).", flush=True)

    def on_log(msg):
        match = SUBGOAL_LOG.search(msg.msg)
        if not match:
            return
        if state["goal_time"] is None:
            print("Thấy sub-goal nhưng chưa bắt được goal thật; không thể tính thời gian.", flush=True)
            return
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        subgoal_time = stamp.to_sec()
        elapsed = subgoal_time - state["goal_time"]
        if elapsed < 0:
            print("ROS time đã reset sau goal; không thể tính thời gian hợp lệ.", flush=True)
            return
        print(
            f"SUB-GOAL ĐẦU TIÊN: cell=({match.group(1)}, {match.group(2)}), "
            f"ROS time={subgoal_time:.3f} s, sau goal={elapsed:.3f} s.",
            flush=True,
        )
        rospy.signal_shutdown("Đã ghi nhận sub-goal đầu tiên")

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, on_goal, queue_size=10)
    rospy.Subscriber("/rosout_agg", Log, on_log, queue_size=100)
    print("Đã đăng ký theo dõi goal và log sub-goal. Có thể bắt đầu chạy main/path_trace_node.", flush=True)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
