"""Loads robots.yaml, the robot settings shared by the sim and the real robots."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory


def config_path():
    """LAB_ROBOTS_CONFIG if set, else the robots.yaml installed with lab_controllers."""
    return os.environ.get('LAB_ROBOTS_CONFIG') or os.path.join(
        get_package_share_directory('lab_controllers'), 'config', 'robots.yaml')


def load_config(path=None):
    path = path or config_path()
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f) or {}
    if not cfg.get('robots'):
        raise ValueError(f'No robots listed in {path}')
    return cfg


def follower_params(cfg, follower=None):
    """Return (follower, target, params) for target_follower.

    follower: robot name to use instead of the file's 'follower' setting.
    """
    robots = cfg['robots']
    follower = follower or str(cfg.get('follower', ''))
    if follower not in robots:
        raise ValueError(f'"{follower}" is not a robot in {config_path()}: {", ".join(robots)}')

    others = [n for n in robots if n != follower]
    target = cfg.get('follower_target') or (others[0] if len(others) == 1 else None)
    if target not in robots or target == follower:
        raise ValueError('With more than two robots, set follower_target in robots.yaml')

    params = {'angular_sign': float(cfg.get('angular_sign', -1.0))}
    params.update(cfg.get('follower_params') or {})   # may still override angular_sign
    params.update({
        'robot_id': int(robots[follower]['rigid_body_id']),
        'target_id': int(robots[target]['rigid_body_id']),
        'mocap_topic': cfg.get('mocap_topic', '/mocap_rigid_bodies'),
        'cmd_vel_topic': robots[follower]['cmd_vel_topic'],
        'cmd_vel_stamped': bool(robots[follower].get('cmd_vel_stamped', False)),
    })
    return follower, target, params


def default_teleop_robot(cfg):
    """The robot that isn't the follower (the one usually driven by hand)."""
    robots = cfg['robots']
    others = [n for n in robots if n != cfg.get('follower')]
    return others[0] if others else next(iter(robots))


def teleop_settings(cfg, name):
    """Topic, message type and speeds for driving robot `name` by hand."""
    robots = cfg['robots']
    if name not in robots:
        raise ValueError(f'"{name}" is not a robot in {config_path()}: {", ".join(robots)}')
    speeds = cfg.get('teleop') or {}
    per_robot = speeds.get(name) or {}
    return {
        'robot_name': name,
        'cmd_vel_topic': robots[name]['cmd_vel_topic'],
        'cmd_vel_stamped': bool(robots[name].get('cmd_vel_stamped', False)),
        'linear_speed': float(per_robot.get('linear_speed', speeds.get('linear_speed', 0.3))),
        'angular_speed': float(per_robot.get('angular_speed', speeds.get('angular_speed', 1.0))),
    }
