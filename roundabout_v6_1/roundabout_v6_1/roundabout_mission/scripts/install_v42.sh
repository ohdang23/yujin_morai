#!/usr/bin/env bash
# Run inside the existing Noetic container; this never launches the vehicle.
set -eo pipefail
WS=/root/morai_ws
PKG="$WS/src/ROI/src/control/roundabout_mission"
ORIGINAL_CONFIG="${1:-/root/roundabout_mission_v41.yaml}"
if [ ! -f "$ORIGINAL_CONFIG" ] && [ "$#" -eq 0 ]; then
  ORIGINAL_CONFIG=/root/roundabout_mission_v4.yaml
fi
test -f "$ORIGINAL_CONFIG"
source /opt/ros/noetic/setup.bash
cd "$WS"
catkin_make --pkg roundabout_mission
source "$WS/devel/setup.bash"
rospack profile >/dev/null
RESOLVED_PACKAGE="$(rospack find roundabout_mission)"
if [ "$(readlink -f "$RESOLVED_PACKAGE")" != "$(readlink -f "$PKG")" ]; then
  echo "ERROR: ROS resolves another package: $RESOLVED_PACKAGE" >&2
  exit 1
fi
python3 - <<'PY'
import os
from roundabout_mission import VERSION, core
assert VERSION == '4.2.0', (VERSION, core.__file__)
assert os.path.realpath(core.__file__).startswith('/root/morai_ws/src/ROI/src/control/roundabout_mission/'), core.__file__
print('VERIFIED', VERSION, os.path.realpath(core.__file__))
PY
CONFIG_OUTPUT="/root/roundabout_mission_v42_$(date +%Y%m%d_%H%M%S).yaml"
rosrun roundabout_mission prepare_roundabout_v4.py --existing "$ORIGINAL_CONFIG" --output "$CONFIG_OUTPUT"
cp "$CONFIG_OUTPUT" /root/roundabout_mission_v42.yaml
printf '%s\n' 'Installed 4.2.0; config: /root/roundabout_mission_v42.yaml'
