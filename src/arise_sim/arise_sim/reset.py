#!/usr/bin/env python3
"""Put sim robots back at their start positions from sim.yaml.

Usage:  ros2 run arise_sim reset              (all robots)
        ros2 run arise_sim reset jackal       (just one)

Uses Gazebo's set_pose service, so the sim keeps running and the ROS nodes
(follower, simulated OptiTrack) are unaffected. A following robot starts
following again right away.
"""
import math
import os
import subprocess
import sys

import yaml
from ament_index_python.packages import get_package_share_directory

WORLD_NAME = 'arise_lab'


def main():
    config = os.path.join(get_package_share_directory('arise_sim'), 'config', 'sim.yaml')
    with open(config, 'r') as f:
        robots = yaml.safe_load(f)['robots']

    names = [a for a in sys.argv[1:] if not a.startswith('--ros-args')] or list(robots)
    unknown = [n for n in names if n not in robots]
    if unknown:
        print(f'Unknown robot(s): {", ".join(unknown)}. Robots in sim.yaml: {", ".join(robots)}')
        sys.exit(1)

    failed = False
    for name in names:
        x, y, yaw = (float(v) for v in robots[name].get('spawn', [0.0, 0.0, 0.0]))
        req = (f"name: '{name}', position: {{x: {x}, y: {y}, z: 0.02}}, "
               f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}")
        cmd = ['gz', 'service', '-s', f'/world/{WORLD_NAME}/set_pose',
               '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
               '--timeout', '3000', '--req', req]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        except (FileNotFoundError, subprocess.TimeoutExpired) as e:
            print(f'{name}: could not reach Gazebo ({e}). Is the simulation running?')
            failed = True
            continue
        if 'data: true' in out.stdout:
            print(f'{name}: reset to x={x:g}, y={y:g}, yaw={yaw:g}')
        else:
            print(f'{name}: reset failed. Is the simulation running?\n{out.stdout}{out.stderr}'.rstrip())
            failed = True
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
