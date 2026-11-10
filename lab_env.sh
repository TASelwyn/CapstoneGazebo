# Run this in every new terminal before using the sim natively:
#   source lab_env.sh
_LAB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "$_LAB_ROOT/install/setup.bash" ]]; then
  source "$_LAB_ROOT/install/setup.bash"
else
  echo "Not built yet: run ./scripts/setup_native.sh first."
fi
# Keep ROS traffic on this computer. The sim publishes /mocap_rigid_bodies,
# the same topic as the real lab, so it must never reach real robots.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
echo "Lab sim environment ready (ROS traffic limited to this computer)."
unset _LAB_ROOT
