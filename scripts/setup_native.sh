#!/usr/bin/env bash
# One-time native setup on Ubuntu 24.04: installs ROS 2 Jazzy + Gazebo Harmonic
# (if missing) and builds the workspace.
#   ./scripts/setup_native.sh
# Safe to run again; it skips what's already done. Asks for your password (sudo).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. /etc/os-release
CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
if [[ "$CODENAME" != "noble" ]]; then
  echo "This script needs Ubuntu 24.04 (noble); this system is '${PRETTY_NAME:-unknown}'."
  echo "ROS 2 Jazzy only has packages for 24.04. Use the Docker setup instead (see README)."
  exit 1
fi

# 1. ROS 2 Jazzy (official steps from docs.ros.org)
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "==> Installing ROS 2 Jazzy (this takes a while)..."
  sudo apt install -y software-properties-common curl
  sudo add-apt-repository -y universe
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F "tag_name" | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt update
  sudo apt install -y ros-jazzy-desktop
else
  echo "==> ROS 2 Jazzy already installed."
fi

# 2. Gazebo Harmonic + what this project needs
echo "==> Installing Gazebo Harmonic and project dependencies..."
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-xacro python3-tk \
  python3-colcon-common-extensions

# 3. Build
cd "$REPO_ROOT"
set +u; source /opt/ros/jazzy/setup.bash; set -u
echo "==> Building..."
colcon build --symlink-install

cat <<EOF

==> Done. In every new terminal, first run (from $REPO_ROOT):
      source lab_env.sh
    Then:
      ros2 launch arise_sim sim.launch.py   # start the sim
      ros2 run lab_controllers teleop       # WASD driving window
      ros2 run lab_controllers follow       # start the follower
      ros2 run arise_sim reset              # sim robots back to start
EOF
