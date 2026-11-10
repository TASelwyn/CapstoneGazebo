#!/usr/bin/env python3
"""WASD keyboard driving in a small window. Works for sim and real robots.

Usage:  ros2 run lab_controllers teleop              (the robot that isn't the follower)
        ros2 run lab_controllers teleop jackal       (choose the robot)

The robot's topic, message type and speeds come from robots.yaml. Any of them
can be overridden with ROS parameters, e.g.
        ros2 run lab_controllers teleop jackal --ros-args -p linear_speed:=0.1

Parameters (override robots.yaml):
    cmd_vel_topic    topic to drive on
    cmd_vel_stamped  true = TwistStamped (Clearpath on Jazzy), false = Twist
    linear_speed     m/s while W or S is held
    angular_speed    rad/s while A or D is held
    robot_name       shown in the window title

For a real robot, run it on a laptop on the lab network (same ROS_DOMAIN_ID as
the robot), not over SSH: the window needs a screen.

Click the window, then hold keys:
    W / S   forward / back
    A / D   turn left / right
    W + A   arc forward-left (any combination works)
    Space   stop

A window is used instead of the terminal because terminals only report the
last key held, so combinations like W + A wouldn't work there. Closing the
window, losing focus, or Ctrl+C all stop the robot.
"""
import signal
import sys
import tkinter as tk

import rclpy
import rclpy.signals
import rclpy.utilities
from geometry_msgs.msg import Twist, TwistStamped

from lab_controllers.robot_config import default_teleop_robot, load_config, teleop_settings

RATE_MS = 50        # publish every 50 ms (20 Hz) while a key is held
RELEASE_MS = 60     # ignore key-ups followed by a key-down within this time (auto-repeat)
DRIVE_KEYS = ('w', 'a', 's', 'd')


class Teleop:

    def __init__(self, root, node, name, topic, stamped, linear, angular):
        self.root, self.node = root, node
        self.linear, self.angular = linear, angular
        self.stamped = stamped
        self.pub = node.create_publisher(TwistStamped if stamped else Twist, topic, 10)
        self.held = set()
        self.pending_release = {}
        self.stop_burst = 0   # zero commands still to send after letting go

        root.title(f'Drive {name}' if name else 'Drive')
        root.configure(padx=16, pady=12)
        tk.Label(root, text=f'Driving: {name or topic}', font=('TkDefaultFont', 13, 'bold')).pack()
        tk.Label(root, text=f'{topic} ({"TwistStamped" if stamped else "Twist"})', fg='#666').pack()
        tk.Label(root, justify='left', font=('TkFixedFont', 11), text=(
            '\nClick this window, then hold:\n'
            '  W / S   forward / back\n'
            '  A / D   turn left / right\n'
            '  W + A   arc (combine any keys)\n'
            '  Space   stop\n')).pack(anchor='w')
        self.status = tk.Label(root, font=('TkFixedFont', 11), width=34, anchor='w')
        self.status.pack()

        root.bind('<KeyPress>', self.on_press)
        root.bind('<KeyRelease>', self.on_release)
        root.bind('<FocusOut>', lambda e: self.clear())   # never leave a key "stuck"
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.tick()

    def on_press(self, event):
        key = event.keysym.lower()
        if key in self.pending_release:            # auto-repeat: cancel the fake release
            self.root.after_cancel(self.pending_release.pop(key))
        if key == 'space':
            self.clear()
        elif key in DRIVE_KEYS:
            self.held.add(key)

    def on_release(self, event):
        key = event.keysym.lower()
        if key in DRIVE_KEYS:
            if key in self.pending_release:
                self.root.after_cancel(self.pending_release[key])
            self.pending_release[key] = self.root.after(RELEASE_MS, self._release, key)

    def _release(self, key):
        self.pending_release.pop(key, None)
        self.held.discard(key)

    def clear(self):
        for job in self.pending_release.values():
            self.root.after_cancel(job)
        self.pending_release.clear()
        self.held.clear()

    def command(self):
        v = self.linear * (('w' in self.held) - ('s' in self.held))
        w = self.angular * (('a' in self.held) - ('d' in self.held))
        return v, w

    def publish(self, v, w):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            twist = msg.twist
        else:
            msg = twist = Twist()
        twist.linear.x, twist.angular.z = float(v), float(w)
        self.pub.publish(msg)

    def tick(self):
        v, w = self.command()
        if v or w:
            self.publish(v, w)
            self.stop_burst = 3
        elif self.stop_burst:
            # Send a few stops after letting go, then go quiet so this window
            # doesn't fight other code (e.g. the follower) driving the robot.
            self.publish(0.0, 0.0)
            self.stop_burst -= 1
        keys = '+'.join(k.upper() for k in DRIVE_KEYS if k in self.held) or '-'
        self.status.config(text=f'keys: {keys:<8} v={v:+.2f} m/s  w={w:+.2f} rad/s')
        self.root.after(RATE_MS, self.tick)

    def send_stop(self):
        try:
            self.clear()
        except tk.TclError:   # window already gone
            self.held.clear()
        for _ in range(3):
            self.publish(0.0, 0.0)

    def close(self):
        self.send_stop()
        self.root.destroy()


def main(args=None):
    # Handle Ctrl+C ourselves so the robot is always told to stop on exit.
    # Robot settings from robots.yaml; ROS parameters below can override them.
    argv = sys.argv if args is None else args
    names = [a for a in rclpy.utilities.remove_ros_args(argv)[1:] if not a.startswith('-')]
    try:
        cfg = load_config()
        defaults = teleop_settings(cfg, names[0] if names else default_teleop_robot(cfg))
    except (ValueError, OSError) as e:
        print(e)
        sys.exit(1)

    rclpy.init(args=args, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = rclpy.create_node(f'teleop_{defaults["robot_name"]}')
    topic = node.declare_parameter('cmd_vel_topic', defaults['cmd_vel_topic']).value
    stamped = bool(node.declare_parameter('cmd_vel_stamped', defaults['cmd_vel_stamped']).value)
    linear = float(node.declare_parameter('linear_speed', defaults['linear_speed']).value)
    angular = float(node.declare_parameter('angular_speed', defaults['angular_speed']).value)
    name = node.declare_parameter('robot_name', defaults['robot_name']).value

    try:
        root = tk.Tk()
    except tk.TclError as e:
        print(f'Could not open the teleop window ({e}).\n'
              'It needs a screen: run it on a laptop/desktop, not over plain SSH. '
              'In Docker, the window appears in the browser view (http://localhost:6080).',
              flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    app = Teleop(root, node, name, topic, stamped, linear, angular)
    signal.signal(signal.SIGINT, lambda *_: root.after(0, app.close))
    signal.signal(signal.SIGTERM, lambda *_: root.after(0, app.close))
    print(f'Teleop window open for {name or topic}. Close the window (or Ctrl+C here) to stop.',
          flush=True)
    try:
        root.mainloop()
    finally:
        app.send_stop()   # covers any exit path; harmless if already sent
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
