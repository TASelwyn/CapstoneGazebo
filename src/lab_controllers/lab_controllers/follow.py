#!/usr/bin/env python3
"""Start target_follower with the settings in robots.yaml. Sim or real robots.

Usage:  ros2 run lab_controllers follow                (follower from robots.yaml)
        ros2 run lab_controllers follow wave_rover     (choose the follower)

Runs in the foreground; Ctrl+C stops it. On exit it also sends the robot a
stop command, because the original script's own stop-on-exit doesn't get
through (see README); otherwise a robot keeps its last command until its own
timeout (real) or forever (Gazebo).
"""
import subprocess
import sys
import time

import rclpy
import rclpy.signals
from geometry_msgs.msg import Twist, TwistStamped

from lab_controllers.robot_config import follower_params, load_config


def _ros_value(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, float):
        return repr(v)
    return str(v)


def _follower_already_running(node):
    deadline = time.time() + 1.5  # give discovery a moment
    while time.time() < deadline:
        if 'target_follower' in node.get_node_names():
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return False


def _send_stop(node, pub, stamped):
    for _ in range(5):
        msg = TwistStamped() if stamped else Twist()
        if stamped:
            msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(0.05)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    try:
        follower, target, params = follower_params(load_config(), args[0] if args else None)
    except (ValueError, OSError) as e:
        print(e)
        sys.exit(1)

    # Handle Ctrl+C ourselves: rclpy's own handler would shut ROS down before
    # we get to send the stop command.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = rclpy.create_node('follow_helper')
    if _follower_already_running(node):
        print('A target_follower is already running. Not starting another.', flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    # Created now, so it's connected and ready the moment we need it.
    stop_pub = node.create_publisher(
        TwistStamped if params['cmd_vel_stamped'] else Twist, params['cmd_vel_topic'], 10)

    cmd = ['ros2', 'run', 'lab_controllers', 'target_follower', '--ros-args',
           '-r', '__node:=target_follower']
    for k, v in params.items():
        cmd += ['-p', f'{k}:={_ros_value(v)}']

    print(f'{follower} (ID {params["robot_id"]}) now follows {target} (ID {params["target_id"]}). '
          f'Press Ctrl+C to stop.', flush=True)
    child = subprocess.Popen(cmd)
    try:
        child.wait()
    except KeyboardInterrupt:
        # Ctrl+C reached the follower too; give it a moment to exit.
        try:
            child.wait(timeout=3)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            child.terminate()
            child.wait()
    finally:
        _send_stop(node, stop_pub, params['cmd_vel_stamped'])
        print(f'Follower stopped; {follower} told to stop.', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
