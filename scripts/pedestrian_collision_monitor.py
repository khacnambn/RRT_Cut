#!/usr/bin/env python3
"""Watch the robot-to-pedestrian distance during one navigation trial.

Every PedestrianSFMPlugin actor publishes its own pose on /<actor>/pose in
Gazebo world coordinates (the plugin stamps them with frame_id "map").  This
node compares those poses with the robot pose and reports a collision as soon
as the planar centre distance drops below --threshold.

The robot pose comes from Gazebo ground truth (/gazebo/model_states) so the
verdict does not inherit AMCL localisation error; tf (map -> base_frame) is
used as a fallback when model_states never arrives.

Collision counting only starts once the robot has moved --arm-move-m away from
where it stood when this node came up, so a pedestrian walking into a robot
that is still waiting for its goal is not charged to the planner.

The summary file is rewritten while the trial runs, so a caller that kills this
node still finds the final numbers on disk.

Usage:
  python3 pedestrian_collision_monitor.py --actors human_1,human_2 \
      --robot-model turtlebot3_waffle --threshold 0.5 \
      --output summary.yaml [--trace distances.csv]
"""

import argparse
import math
import os
import signal
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped

TOPIC_WAIT_S = 10.0   # how long to wait for the first pose of every actor
GAZEBO_GRACE_S = 3.0  # how long Gazebo ground truth gets before tf is tried


def _raise_keyboard_interrupt(signum, frame):
    """SIGTERM must flush the summary too, not just SIGINT."""
    raise KeyboardInterrupt


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--actors", default="",
                   help="comma separated actor names; empty means auto-discover /<name>/pose topics")
    p.add_argument("--robot-model", default="turtlebot3_waffle",
                   help="Gazebo model name of the robot in /gazebo/model_states")
    p.add_argument("--robot-source", default="auto", choices=("auto", "gazebo", "tf"),
                   help="where the robot pose comes from")
    p.add_argument("--base-frame", default="base_footprint", help="robot frame for the tf fallback")
    p.add_argument("--map-frame", default="map", help="reference frame for the tf fallback")
    p.add_argument("--threshold", type=float, default=0.5,
                   help="centre distance (m) below which the trial counts as a collision")
    p.add_argument("--rearm-margin", type=float, default=0.1,
                   help="extra distance (m) to clear before the same actor can raise a new event")
    p.add_argument("--arm-move-m", type=float, default=0.05,
                   help="robot displacement (m) required before collisions are counted")
    p.add_argument("--rate", type=float, default=20.0, help="sampling rate (Hz)")
    p.add_argument("--stale-s", type=float, default=1.0,
                   help="ignore an actor pose older than this (s)")
    p.add_argument("--output", required=True, help="summary file, rewritten while the trial runs")
    p.add_argument("--trace", default="", help="optional CSV of every sampled distance")
    return p.parse_args(argv)


def discover_actors():
    """Actor pose topics look like /human_1/pose and carry a plain PoseStamped."""
    found = []
    for name, msg_type in rospy.get_published_topics():
        if msg_type != "geometry_msgs/PoseStamped":
            continue
        parts = name.strip("/").split("/")
        if len(parts) == 2 and parts[1] == "pose":
            found.append(parts[0])
    return sorted(set(found))


class RobotPose:
    """Robot position in the Gazebo world frame, from model_states or tf."""

    def __init__(self, args):
        self.args = args
        self.xy = None
        self.source = None
        self._sub = None
        self._tf_buffer = None
        self._tf_listener = None

        if args.robot_source in ("auto", "gazebo"):
            try:
                from gazebo_msgs.msg import ModelStates
                self._sub = rospy.Subscriber("/gazebo/model_states", ModelStates,
                                             self._on_model_states, queue_size=1)
            except ImportError:
                rospy.logwarn("gazebo_msgs not available, falling back to tf")

        if args.robot_source == "tf":
            self._start_tf()

    def _start_tf(self):
        if self._tf_buffer is not None:
            return
        import tf2_ros
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def _on_model_states(self, msg):
        try:
            index = msg.name.index(self.args.robot_model)
        except ValueError:
            # Fall back to the first model that is neither scenery nor an actor.
            index = None
            for i, name in enumerate(msg.name):
                if name in ("ground_plane", "sun") or name.startswith("human"):
                    continue
                if "turtlebot" in name or "robot" in name or "waffle" in name or "burger" in name:
                    index = i
                    break
            if index is None:
                return
        position = msg.pose[index].position
        self.xy = (position.x, position.y)
        self.source = "gazebo"

    def update(self):
        """Return the current (x, y), switching to tf if model_states stays silent."""
        if self.xy is not None and self.source == "gazebo":
            return self.xy
        if self._tf_buffer is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(self.args.map_frame, self.args.base_frame,
                                                  rospy.Time(0), rospy.Duration(0.05))
        except Exception:
            return None
        self.xy = (tf.transform.translation.x, tf.transform.translation.y)
        self.source = "tf"
        return self.xy

    def enable_tf_fallback(self):
        if self.args.robot_source == "gazebo":
            return
        self._start_tf()


class Monitor:
    def __init__(self, args, actors):
        self.args = args
        self.actors = actors
        self.poses = {}  # actor -> (x, y, wall time of arrival)
        self.in_collision = {a: False for a in actors}
        self.min_dist = {a: float("inf") for a in actors}
        self.events = []  # (actor, t, distance)
        self.samples = 0
        self.armed = False
        self.arm_time = None
        self.start_xy = None
        self.start_wall = time.time()
        self.robot = RobotPose(args)
        self.trace = None
        self._last_write = 0.0

        if args.trace:
            self.trace = open(args.trace, "w", buffering=1)
            self.trace.write("t_s,armed,robot_x,robot_y,min_dist_m," +
                             ",".join("d_" + a for a in actors) + "\n")

        for actor in actors:
            rospy.Subscriber("/%s/pose" % actor, PoseStamped, self._on_pose,
                             callback_args=actor, queue_size=1)

    def _on_pose(self, msg, actor):
        self.poses[actor] = (msg.pose.position.x, msg.pose.position.y, time.time())

    def elapsed(self):
        return time.time() - self.start_wall

    def step(self):
        robot_xy = self.robot.update()
        if robot_xy is None:
            return

        if self.start_xy is None:
            self.start_xy = robot_xy
        if not self.armed:
            moved = math.hypot(robot_xy[0] - self.start_xy[0], robot_xy[1] - self.start_xy[1])
            if moved >= self.args.arm_move_m:
                self.armed = True
                self.arm_time = self.elapsed()
                rospy.loginfo("collision monitor armed after %.2f m (t=%.1fs)", moved, self.arm_time)

        now = time.time()
        distances = {}
        for actor in self.actors:
            sample = self.poses.get(actor)
            if sample is None or now - sample[2] > self.args.stale_s:
                continue
            distances[actor] = math.hypot(robot_xy[0] - sample[0], robot_xy[1] - sample[1])

        if distances:
            self.samples += 1
        t = self.elapsed()

        for actor, dist in distances.items():
            if self.armed and dist < self.min_dist[actor]:
                self.min_dist[actor] = dist
            if not self.armed:
                continue
            if dist < self.args.threshold:
                if not self.in_collision[actor]:
                    self.in_collision[actor] = True
                    self.events.append((actor, t, dist))
                    # The runner greps this line to end the trial early.
                    print("COLLISION actor=%s dist=%.4f t=%.2f" % (actor, dist, t), flush=True)
                    rospy.logwarn("COLLISION with %s at %.3f m (t=%.1fs)", actor, dist, t)
            elif dist > self.args.threshold + self.args.rearm_margin:
                self.in_collision[actor] = False

        if self.trace is not None:
            row = ["%.3f" % t, "1" if self.armed else "0",
                   "%.4f" % robot_xy[0], "%.4f" % robot_xy[1]]
            finite = list(distances.values())
            row.append("%.4f" % min(finite) if finite else "")
            for actor in self.actors:
                row.append("%.4f" % distances[actor] if actor in distances else "")
            self.trace.write(",".join(row) + "\n")

        if now - self._last_write >= 1.0:
            self.write_summary()
            self._last_write = now

    def overall_min(self):
        best_actor, best = "", float("inf")
        for actor, dist in self.min_dist.items():
            if dist < best:
                best_actor, best = actor, dist
        return best_actor, best

    def write_summary(self):
        best_actor, best = self.overall_min()
        lines = [
            "threshold_m: %.4f" % self.args.threshold,
            "arm_move_m: %.4f" % self.args.arm_move_m,
            "actors: %s" % ",".join(self.actors),
            "robot_pose_source: %s" % (self.robot.source or "none"),
            "armed: %d" % (1 if self.armed else 0),
            "arm_time_s: %s" % ("%.2f" % self.arm_time if self.arm_time is not None else "nan"),
            "samples: %d" % self.samples,
            "duration_s: %.2f" % self.elapsed(),
            "min_distance_m: %s" % ("%.4f" % best if best != float("inf") else "nan"),
            "min_distance_actor: %s" % (best_actor or "none"),
            "collision_events: %d" % len(self.events),
        ]
        for actor in self.actors:
            dist = self.min_dist[actor]
            lines.append("min_distance_%s_m: %s" % (actor, "%.4f" % dist if dist != float("inf") else "nan"))
        if self.events:
            actor, t, dist = self.events[0]
            lines.append("first_collision_actor: %s" % actor)
            lines.append("first_collision_time_s: %.2f" % t)
            lines.append("first_collision_distance_m: %.4f" % dist)
        text = "\n".join(lines) + "\n"

        tmp = self.args.output + ".tmp"
        with open(tmp, "w") as handle:
            handle.write(text)
        os.replace(tmp, self.args.output)

    def close(self):
        self.write_summary()
        if self.trace is not None:
            self.trace.close()


def main(argv):
    args = parse_args(rospy.myargv(argv)[1:])
    rospy.init_node("pedestrian_collision_monitor", anonymous=True, disable_signals=True)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    actors = [a for a in (x.strip() for x in args.actors.split(",")) if a]
    if not actors:
        deadline = time.time() + 5.0
        while not actors and time.time() < deadline:
            actors = discover_actors()
            time.sleep(0.5)
    if not actors:
        rospy.logerr("no pedestrian pose topics found; this is not a dynamic scene")
        return 2

    rospy.loginfo("watching pedestrians: %s (threshold %.2f m)", ",".join(actors), args.threshold)
    monitor = Monitor(args, actors)
    monitor.write_summary()

    # Everything below runs under one finally: a trial is ended by killing this
    # node, and the caller reads the summary file right afterwards.
    try:
        # Wait for the first pose of each actor, then for the robot pose, before
        # sampling starts; otherwise the first seconds produce no distances.
        deadline = time.time() + TOPIC_WAIT_S
        while time.time() < deadline and not rospy.is_shutdown():
            if len(monitor.poses) == len(actors):
                break
            time.sleep(0.2)
        missing = [a for a in actors if a not in monitor.poses]
        if missing:
            rospy.logwarn("no pose yet from: %s", ",".join(missing))

        started = time.time()
        deadline = started + TOPIC_WAIT_S
        while time.time() < deadline and not rospy.is_shutdown():
            if monitor.robot.update() is not None:
                break
            if time.time() > started + GAZEBO_GRACE_S:
                monitor.robot.enable_tf_fallback()
            time.sleep(0.2)
        if monitor.robot.update() is None:
            rospy.logerr("no robot pose from Gazebo model_states or tf")
            return 3

        period = 1.0 / max(args.rate, 1.0)
        # A wall-clock loop: /clock stalling must not freeze the monitor.
        while not rospy.is_shutdown():
            monitor.step()
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
