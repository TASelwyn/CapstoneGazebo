#!/usr/bin/env python3
"""Simulated OptiTrack.

Reads each robot's true pose from Gazebo and republishes it exactly the way
the lab's OptiTrack driver (TU Delft ros2-mocap_optitrack) does:
mocap_optitrack_interfaces/RigidBodyArray on /mocap_rigid_bodies, one
RigidBody per robot, sorted by Streaming ID.

Code written against the real driver (e.g. target_follower.py) runs unchanged.
Rigid-body IDs and the topic come from lab_controllers' robots.yaml (shared
with the real robots); frame, noise and rate come from sim.yaml.
"""
import math
import random

import rclpy
import yaml
from lab_controllers.robot_config import config_path, load_config
from mocap_optitrack_interfaces.msg import RigidBody, RigidBodyArray
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


# --- small rotation helpers (3x3 matrices as lists of rows) ------------------

def quat_to_mat(x, y, z, w):
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def mat_to_quat(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (m[2][1] - m[1][2]) / s
        y, z = (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w, x = (m[2][1] - m[1][2]) / s, 0.25 * s
        y, z = (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w, x = (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s
        y, z = 0.25 * s, (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w, x = (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s
        y, z = (m[1][2] + m[2][1]) / s, 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return x / n, y / n, z / n, w / n


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


class MocapFrame:
    """Maps sim-world poses into the (configurable) OptiTrack frame."""

    def __init__(self, cfg):
        self.origin = [float(v) for v in cfg.get('origin', [0.0, 0.0, 0.0])]
        self.r_world_to_local = rot_z(-float(cfg.get('origin_yaw', 0.0)))
        mirror = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
        identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        # Z-up (ROS) -> Y-up (Motive): x stays, y_out = z, z_out = -y
        z_to_y_up = [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]
        up_axis = str(cfg.get('up_axis', 'z')).lower()
        if up_axis not in ('z', 'y'):
            raise ValueError(f"mocap.frame.up_axis must be 'z' or 'y', got {up_axis!r}")
        s = mirror if bool(cfg.get('mirror_y', False)) else identity
        a = z_to_y_up if up_axis == 'y' else identity
        # Linear map applied after moving into the mocap origin frame.
        self.l_map = matmul(a, s)
        self.l_map_t = transpose(self.l_map)

    def apply(self, pos, rot):
        p = [pos[i] - self.origin[i] for i in range(3)]
        p = matvec(self.r_world_to_local, p)
        r = matmul(self.r_world_to_local, rot)
        # Conjugating by the map keeps r a proper rotation even when mirrored.
        return matvec(self.l_map, p), matmul(matmul(self.l_map, r), self.l_map_t)


class MocapSim(Node):

    def __init__(self):
        super().__init__('mocap_sim')
        self.declare_parameter('sim_config', '')
        path = self.get_parameter('sim_config').value
        if not path:
            raise RuntimeError('mocap_sim needs the sim_config parameter (sim.yaml)')
        with open(path, 'r') as f:
            sim_cfg = yaml.safe_load(f)
        shared = load_config()   # robots.yaml: rigid-body IDs and the mocap topic

        mcfg = sim_cfg.get('mocap', {})
        self.frame = MocapFrame(mcfg.get('frame', {}))
        self.mean_error = float(mcfg.get('mean_error', 0.0005))
        self.pos_noise = float(mcfg.get('position_noise_std', 0.0))
        self.yaw_noise = float(mcfg.get('yaw_noise_std', 0.0))

        self.bodies = {}   # robot name -> dict(id, offset, pose)
        for name, rcfg in sim_cfg['robots'].items():
            if name not in shared['robots']:
                raise RuntimeError(f'Robot "{name}" in sim.yaml is missing from {config_path()}')
            body = {
                'id': int(shared['robots'][name]['rigid_body_id']),
                'offset': [float(v) for v in rcfg.get('pivot_offset', [0.0, 0.0, 0.0])],
                'pose': None,
            }
            self.bodies[name] = body
            self.create_subscription(
                Odometry, f'/sim/{name}/ground_truth',
                lambda msg, b=body: self._on_truth(b, msg), 10)

        ids = [b['id'] for b in self.bodies.values()]
        if len(ids) != len(set(ids)):
            raise RuntimeError(f'Duplicate rigid_body_id in {config_path()}: {ids}')

        self.pub = self.create_publisher(
            RigidBodyArray, shared.get('mocap_topic', '/mocap_rigid_bodies'), 10)
        rate = float(mcfg.get('rate_hz', 120.0))
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            'Simulated OptiTrack: ' + ', '.join(f'{n} -> ID {b["id"]}' for n, b in self.bodies.items())
            + f' on {self.pub.topic_name} at {rate:g} Hz')

    def _on_truth(self, body, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        body['pose'] = ([p.x, p.y, p.z], quat_to_mat(q.x, q.y, q.z, q.w))

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        out = RigidBodyArray()
        out.header.stamp = stamp
        for body in sorted(self.bodies.values(), key=lambda b: b['id']):
            if body['pose'] is None:
                continue  # no data from Gazebo yet; real Motive also omits untracked bodies
            pos, rot = body['pose']
            # Move from the robot's floor centre to the rigid-body pivot.
            off = matvec(rot, body['offset'])
            pos = [pos[i] + off[i] for i in range(3)]
            if self.pos_noise > 0.0:
                pos = [pos[0] + random.gauss(0.0, self.pos_noise),
                       pos[1] + random.gauss(0.0, self.pos_noise), pos[2]]
            if self.yaw_noise > 0.0:
                rot = matmul(rot_z(random.gauss(0.0, self.yaw_noise)), rot)
            pos, rot = self.frame.apply(pos, rot)
            qx, qy, qz, qw = mat_to_quat(rot)

            rb = RigidBody()
            rb.header.stamp = stamp
            rb.id = body['id']
            rb.valid = True
            rb.mean_error = self.mean_error
            rb.pose_stamped.header.stamp = stamp
            rb.pose_stamped.pose.position.x = pos[0]
            rb.pose_stamped.pose.position.y = pos[1]
            rb.pose_stamped.pose.position.z = pos[2]
            rb.pose_stamped.pose.orientation.x = qx
            rb.pose_stamped.pose.orientation.y = qy
            rb.pose_stamped.pose.orientation.z = qz
            rb.pose_stamped.pose.orientation.w = qw
            out.rigid_bodies.append(rb)
        if out.rigid_bodies:
            self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MocapSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # normal Ctrl+C / launch shutdown
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
