#!/usr/bin/env python3
"""Drive a robot around randomly inside the arena. Sim or real robots.

Usage:  ros2 run lab_controllers wander               (the robot that isn't the follower)
        ros2 run lab_controllers wander wave_rover    (choose the robot)

Uses OptiTrack to know where the robot is:
  - drives forward, picking a new random turn every few seconds
  - near the arena edge, turns back toward the middle
  - steers away from any other tracked body that gets too close
  - stops if its own OptiTrack data goes missing (lost tracking)
Ctrl+C stops it and the robot. Settings: the wander and arena sections of
robots.yaml.
"""
import math
import random
import sys
import time

import rclpy
import rclpy.signals
import rclpy.utilities
from geometry_msgs.msg import Twist, TwistStamped
from mocap_optitrack_interfaces.msg import RigidBodyArray
from rclpy.node import Node

from lab_controllers.robot_config import config_path, default_teleop_robot, load_config

RATE = 20.0          # control loop, Hz
STALE_AFTER = 0.5    # s without our own mocap data -> stop


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q):  # same formula as target_follower.py
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Wander(Node):

    def __init__(self, name, cfg):
        super().__init__(f'wander_{name}')
        robot = cfg['robots'][name]
        w = cfg.get('wander') or {}
        arena = cfg.get('arena') or {}
        self.robot_name = name
        self.robot_id = int(robot['rigid_body_id'])
        self.stamped = bool(robot.get('cmd_vel_stamped', False))
        self.angular_sign = float(cfg.get('angular_sign', -1.0))
        self.speed = float(w.get('speed', 0.15))
        self.max_turn = float(w.get('max_turn', 0.6))
        self.turn_change = [float(v) for v in w.get('turn_change', [2.0, 5.0])]
        self.margin = float(w.get('wall_margin', 0.6))
        self.avoid_radius = float(w.get('avoid_radius', 0.6))
        (x0, x1), (y0, y1) = arena.get('x', [-2.0, 2.0]), arena.get('y', [-4.0, 4.0])
        self.inner = (x0 + self.margin, x1 - self.margin, y0 + self.margin, y1 - self.margin)
        self.center = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)

        self.pose = None          # (x, y, yaw)
        self.pose_time = 0.0
        self.others = []          # [(x, y)] of other tracked bodies
        self.turn = 0.0
        self.next_turn_change = 0.0
        self.mode = None

        self.pub = self.create_publisher(
            TwistStamped if self.stamped else Twist, robot['cmd_vel_topic'], 10)
        self.create_subscription(RigidBodyArray, cfg.get('mocap_topic', '/mocap_rigid_bodies'),
                                 self.on_mocap, 20)
        self.create_timer(1.0 / RATE, self.step)
        self.get_logger().info(
            f'Wandering {name} (ID {self.robot_id}) on {robot["cmd_vel_topic"]}, '
            f'speed {self.speed} m/s, arena x {x0}..{x1}, y {y0}..{y1}')

    def on_mocap(self, msg):
        others = []
        for rb in msg.rigid_bodies:
            p = rb.pose_stamped.pose.position
            if int(rb.id) == self.robot_id:
                self.pose = (p.x, p.y, quat_to_yaw(rb.pose_stamped.pose.orientation))
                self.pose_time = time.monotonic()
            else:
                others.append((p.x, p.y))
        self.others = others

    def publish(self, v, w):
        w *= self.angular_sign   # same convention as target_follower
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            twist = msg.twist
        else:
            msg = twist = Twist()
        twist.linear.x, twist.angular.z = float(v), float(w)
        self.pub.publish(msg)

    def set_mode(self, mode):
        if mode != self.mode:
            self.mode = mode
            self.get_logger().info(mode)

    def head_toward(self, heading):
        """Command that turns toward a heading; drives only when roughly facing it."""
        x, y, yaw = self.pose
        err = wrap_pi(heading - yaw)
        w = max(-self.max_turn * 1.5, min(self.max_turn * 1.5, 1.5 * err))
        v = self.speed * max(0.0, math.cos(err)) if abs(err) < math.pi / 2 else 0.0
        return v, w

    def step(self):
        now = time.monotonic()
        if self.pose is None or now - self.pose_time > STALE_AFTER:
            self.set_mode('No OptiTrack data for this robot: stopped')
            self.publish(0.0, 0.0)
            return
        x, y, yaw = self.pose

        # 1. Too close to another tracked body: steer directly away from it.
        near = [(ox, oy) for ox, oy in self.others
                if math.hypot(ox - x, oy - y) < self.avoid_radius]
        if near:
            ox, oy = min(near, key=lambda o: math.hypot(o[0] - x, o[1] - y))
            self.set_mode('Avoiding a nearby body')
            self.publish(*self.head_toward(math.atan2(y - oy, x - ox)))
            return

        # 2. Near the arena edge: head back toward the middle.
        xi0, xi1, yi0, yi1 = self.inner
        if not (xi0 <= x <= xi1 and yi0 <= y <= yi1):
            self.set_mode('Near the edge: heading back to the middle')
            self.publish(*self.head_toward(math.atan2(self.center[1] - y, self.center[0] - x)))
            self.next_turn_change = 0.0   # pick a fresh random turn once back inside
            return

        # 3. Free space: drive with a random turn, changed every few seconds.
        if now >= self.next_turn_change:
            self.turn = random.uniform(-self.max_turn, self.max_turn)
            self.next_turn_change = now + random.uniform(*self.turn_change)
        self.set_mode('Wandering')
        self.publish(self.speed, self.turn)


def main(args=None):
    argv = sys.argv if args is None else args
    names = [a for a in rclpy.utilities.remove_ros_args(argv)[1:] if not a.startswith('-')]
    try:
        cfg = load_config()
        name = names[0] if names else default_teleop_robot(cfg)
        if name not in cfg['robots']:
            raise ValueError(f'"{name}" is not a robot in {config_path()}: '
                             f'{", ".join(cfg["robots"])}')
    except (ValueError, OSError) as e:
        print(e)
        sys.exit(1)

    # Handle Ctrl+C ourselves so the robot is always told to stop on exit.
    rclpy.init(args=args, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = Wander(name, cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            node.publish(0.0, 0.0)
            time.sleep(0.05)
        print(f'Wander stopped; {name} told to stop.', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
