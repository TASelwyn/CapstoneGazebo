#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from mocap_optitrack_interfaces.msg import RigidBodyArray

# Lab sim note: the only change from the original lab script is the
# 'cmd_vel_stamped' parameter (default False = original behaviour).
# Clearpath robots on ROS 2 Jazzy expect TwistStamped on cmd_vel.


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TargetFollower(Node):
    def __init__(self):
        super().__init__('target_follower')

        self.declare_parameter('mocap_topic', '/mocap_rigid_bodies')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('robot_id', 3)
        self.declare_parameter('target_id', 2)

        self.declare_parameter('k_rho', 3)
        self.declare_parameter('k_theta', 0.01)

        self.declare_parameter('v_max', 0.18)
        self.declare_parameter('w_max', 0.8)
        self.declare_parameter('w_min', 0.22)

        self.declare_parameter('stop_radius', 1.1)
        self.declare_parameter('resume_radius', 1.6)

        self.declare_parameter('turn_enter_angle', 0.55)
        self.declare_parameter('turn_exit_angle', 0.20)

        self.declare_parameter('require_valid', False)
        self.declare_parameter('max_mean_error', 0.02)

        self.declare_parameter('angular_sign', -1.0)
        self.declare_parameter('cmd_vel_stamped', False)

        self.mocap_topic = self.get_parameter('mocap_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.robot_id = int(self.get_parameter('robot_id').value)
        self.target_id = int(self.get_parameter('target_id').value)

        self.k_rho = float(self.get_parameter('k_rho').value)
        self.k_theta = float(self.get_parameter('k_theta').value)

        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.w_min = float(self.get_parameter('w_min').value)

        self.stop_radius = float(self.get_parameter('stop_radius').value)
        self.resume_radius = float(self.get_parameter('resume_radius').value)

        self.turn_enter_angle = float(self.get_parameter('turn_enter_angle').value)
        self.turn_exit_angle = float(self.get_parameter('turn_exit_angle').value)

        self.require_valid = bool(self.get_parameter('require_valid').value)
        self.max_mean_error = float(self.get_parameter('max_mean_error').value)
        self.angular_sign = float(self.get_parameter('angular_sign').value)
        self.cmd_vel_stamped = bool(self.get_parameter('cmd_vel_stamped').value)

        self.robot_pose = None
        self.target_pos = None
        self.robot_ok = False
        self.target_ok = False

        self.mode = "TURN"

        self.pub = self.create_publisher(
            TwistStamped if self.cmd_vel_stamped else Twist, self.cmd_vel_topic, 10)
        self.sub = self.create_subscription(
            RigidBodyArray,
            self.mocap_topic,
            self.on_mocap,
            20
        )
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"Started target_follower robot_id={self.robot_id} target_id={self.target_id}"
        )

    def on_mocap(self, msg):
        self.robot_ok = False
        self.target_ok = False

        for rb in msg.rigid_bodies:
            ok = True
            if self.require_valid:
                ok = bool(rb.valid) and float(rb.mean_error) <= self.max_mean_error

            p = rb.pose_stamped.pose.position
            q = rb.pose_stamped.pose.orientation

            if int(rb.id) == self.robot_id:
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                self.robot_pose = (p.x, p.y, yaw)
                self.robot_ok = ok

            elif int(rb.id) == self.target_id:
                self.target_pos = (p.x, p.y)
                self.target_ok = ok

    def publish_cmd(self, v, w):
        if self.cmd_vel_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            twist = msg.twist
        else:
            msg = Twist()
            twist = msg
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.pub.publish(msg)

    def publish_stop(self):
        self.publish_cmd(0.0, 0.0)

    def control_loop(self):
        if self.robot_pose is None or self.target_pos is None:
            self.publish_stop()
            return

        if self.require_valid and (not self.robot_ok or not self.target_ok):
            self.publish_stop()
            return

        x, y, yaw = self.robot_pose
        xt, yt = self.target_pos

        dx = xt - x
        dy = yt - y
        rho = math.hypot(dx, dy)
        theta_d = math.atan2(dy, dx)
        e_theta = wrap_pi(theta_d - yaw)

        # --- STOP state logic ---
        if self.mode != "STOP" and rho <= self.stop_radius:
            self.mode = "STOP"

        elif self.mode == "STOP":
            # stay stopped unless target moves away enough
            if rho >= self.resume_radius:
                if abs(e_theta) > self.turn_enter_angle:
                    self.mode = "TURN"
                else:
                    self.mode = "DRIVE"

        # --- normal TURN/DRIVE transitions ---
        elif self.mode == "TURN":
            if abs(e_theta) < self.turn_exit_angle:
                self.mode = "DRIVE"

        elif self.mode == "DRIVE":
            if abs(e_theta) > self.turn_enter_angle:
                self.mode = "TURN"

        else:
            self.mode = "TURN"

        # --- command generation ---
        if self.mode == "STOP":
            v = 0.0
            w = 0.0

        elif self.mode == "TURN":
            v = 0.0
            w = self.k_theta * e_theta

            if abs(w) < self.w_min:
                w = self.w_min if e_theta >= 0.0 else -self.w_min

            w = clamp(w, -self.w_max, self.w_max)

        else:  # DRIVE
            v = clamp(self.k_rho * rho, 0.0, self.v_max)

            # reduce forward speed if heading is off
            heading_scale = max(0.0, math.cos(e_theta))
            v *= heading_scale

            w = clamp(self.k_theta * e_theta, -self.w_max, self.w_max)

        w *= self.angular_sign
        self.publish_cmd(v, w)

        self.get_logger().info(
            f"[{self.mode}] robot=({x:.2f},{y:.2f},{yaw:.2f}) "
            f"target=({xt:.2f},{yt:.2f}) "
            f"rho={rho:.2f} theta_d={theta_d:.2f} eth={e_theta:.2f} "
            f"cmd=({v:.2f},{w:.2f})"
        )

    def destroy_node(self):
        self.publish_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TargetFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
