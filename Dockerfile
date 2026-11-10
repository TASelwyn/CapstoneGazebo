# ARISE lab simulation: ROS 2 Jazzy + Gazebo Harmonic (the official pairing).
# Works on Linux, Windows (Docker Desktop) and Mac (Intel and Apple Silicon).
# The Gazebo window is shown in your web browser via noVNC, so no X server or
# GPU setup is needed on any OS.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ros-jazzy-ros-gz pulls in Gazebo Harmonic itself. The second line is the
# browser display (virtual screen + VNC + noVNC web client).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz ros-jazzy-xacro python3-tk \
        xvfb x11vnc novnc websockify openbox \
        libgl1 libglx-mesa0 libgl1-mesa-dri mesa-utils \
        python3-yaml python3-colcon-common-extensions build-essential cmake \
    && rm -rf /var/lib/apt/lists/*

# Keep ROS traffic inside this computer: the sim publishes /mocap_rigid_bodies,
# the same topic as the real lab, and must never reach real robots.
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
COPY docker/teleop /usr/local/bin/teleop
COPY docker/ros-env /usr/local/bin/ros-env
COPY docker/reset /usr/local/bin/reset
COPY docker/follow /usr/local/bin/follow
COPY docker/wander /usr/local/bin/wander
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/teleop /usr/local/bin/ros-env \
        /usr/local/bin/reset /usr/local/bin/follow /usr/local/bin/wander \
    && echo 'source /opt/ros/jazzy/setup.bash; [ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

# The repo is mounted at /ws at run time (see compose.yaml), so edits on your
# computer are picked up on the next start without rebuilding the image.
WORKDIR /ws
EXPOSE 6080
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["ros2", "launch", "arise_sim", "sim.launch.py"]
