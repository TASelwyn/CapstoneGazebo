from glob import glob

from setuptools import setup

package_name = 'lab_controllers'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Capstone team',
    maintainer_email='todo@example.com',
    description='Lab robot code that runs the same on the sim and the real robots.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'target_follower = lab_controllers.target_follower:main',
            'follow = lab_controllers.follow:main',
            'teleop = lab_controllers.teleop:main',
            'wander = lab_controllers.wander:main',
        ],
    },
)
