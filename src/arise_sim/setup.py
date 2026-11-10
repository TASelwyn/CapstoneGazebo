import os
from glob import glob

from setuptools import setup

package_name = 'arise_sim'


def tree(folder):
    """Install every file under folder, keeping its sub-folder layout."""
    entries = []
    for root, _, files in os.walk(folder):
        if files:
            entries.append((os.path.join('share', package_name, root),
                            [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
    ] + tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Capstone team',
    maintainer_email='todo@example.com',
    description='Simulation-only parts of the ARISE lab: world, robot models, '
                'simulated OptiTrack, launch and reset.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mocap_sim = arise_sim.mocap_sim:main',
            'reset = arise_sim.reset:main',
        ],
    },
)
