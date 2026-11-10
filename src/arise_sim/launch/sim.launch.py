"""Start the lab simulation (sim-only parts).

    ros2 launch arise_sim sim.launch.py

Options (all optional):
    gui:=false                run Gazebo without its window (faster, no display needed)
    obstacles:=true           add the original drone-world boxes and landmarks
    sim_config:=/path/to.yaml use a different sim.yaml

What it starts:
    Gazebo (ARISE cage) -> the robots -> ROS<->Gazebo bridge
    -> simulated OptiTrack (/mocap_rigid_bodies)

Robot IDs and topics come from lab_controllers' robots.yaml, shared with the
real robots, so the sim robots answer on exactly the same topics. Drive or
follow them with the same commands as real robots:
    ros2 run lab_controllers teleop / follow
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from lab_controllers.robot_config import config_path, load_config
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_NAME = 'arise_lab'


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    share = get_package_share_directory('arise_sim')

    sim_config = LaunchConfiguration('sim_config').perform(context)
    with open(sim_config, 'r') as f:
        sim_cfg = yaml.safe_load(f)
    shared = load_config()
    missing = [n for n in sim_cfg['robots'] if n not in shared['robots']]
    if missing:
        raise RuntimeError(f'Robot(s) {missing} in {sim_config} are missing from {config_path()}')

    gui = _truthy(LaunchConfiguration('gui').perform(context))
    actions = [LogInfo(msg=f'Robot settings: {config_path()}')]

    # --- Gazebo --------------------------------------------------------------
    world_file = os.path.join(share, 'worlds', 'arise_lab.sdf')
    gz_args = f'-r {"" if gui else "-s "}{world_file}'
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()))

    # --- Robots ----------------------------------------------------------------
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    for name, sim_r in sim_cfg['robots'].items():
        r = shared['robots'][name]
        urdf = xacro.process_file(
            os.path.join(share, 'urdf', f'{sim_r["model"]}.urdf.xacro'),
            mappings={
                'name': name,
                'cmd_vel_topic': r['cmd_vel_topic'],
                'ground_truth_topic': f'/sim/{name}/ground_truth',
                'ground_truth_rate': str(sim_cfg.get('mocap', {}).get('rate_hz', 120)),
            }).toxml()
        x, y, yaw = (float(v) for v in sim_r.get('spawn', [0.0, 0.0, 0.0]))
        actions.append(Node(
            package='ros_gz_sim', executable='create', name=f'spawn_{name}', output='screen',
            arguments=['-world', WORLD_NAME, '-name', name, '-string', urdf,
                       '-x', str(x), '-y', str(y), '-z', '0.02', '-Y', str(yaw)]))

        twist = 'TwistStamped' if r.get('cmd_vel_stamped', False) else 'Twist'
        bridge_args += [
            f'{r["cmd_vel_topic"]}@geometry_msgs/msg/{twist}]gz.msgs.Twist',
            f'/sim/{name}/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ]

    if _truthy(LaunchConfiguration('obstacles').perform(context)):
        actions.append(Node(
            package='ros_gz_sim', executable='create', name='spawn_obstacles', output='screen',
            arguments=['-world', WORLD_NAME, '-name', 'lab_obstacles', '-file',
                       os.path.join(share, 'models', 'lab_obstacles', 'model.sdf')]))

    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
        output='screen', arguments=bridge_args))

    # --- Simulated OptiTrack ---------------------------------------------------
    actions.append(Node(
        package='arise_sim', executable='mocap_sim', name='mocap_sim', output='screen',
        parameters=[{'sim_config': sim_config, 'use_sim_time': True}]))

    actions.append(LogInfo(msg='Sim ready. Drive: ros2 run lab_controllers teleop | '
                               'Follow: ros2 run lab_controllers follow | '
                               'Reset: ros2 run arise_sim reset'))
    return actions


def generate_launch_description():
    share = get_package_share_directory('arise_sim')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show the Gazebo window'),
        DeclareLaunchArgument('obstacles', default_value='false',
                              description='Add the original obstacle boxes and landmarks'),
        DeclareLaunchArgument('sim_config',
                              default_value=os.path.join(share, 'config', 'sim.yaml'),
                              description='Simulation-only settings file'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        OpaqueFunction(function=_setup),
    ])
